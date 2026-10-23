#include "dvc_supercap.hpp"
#include <math.h>

namespace {
class IrqLock {
public:
    IrqLock() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(mask_); }
private:
    uint32_t mask_;
};
uint16_t readLe16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
bool validInput(float value, float maximum)
{
    return isfinite(value) && value >= 1.0f && value <= maximum && value <= 65535.0f;
}
}

Supercap* Supercap::devices_[3] = {};
uint8_t Supercap::device_count_ = 0;
bool Supercap::observer_registered_ = false;

bool Supercap::init(const SupercapConfig& config)
{
    if (initialized_ || config.bus == nullptr || config.rx_identifier > 0x7FFU
        || !validInput(config.max_referee_power_w, 65535.0f)
        || !validInput(config.max_buffer_j, 65535.0f)) { return false; }
    IrqLock lock;
    if (device_count_ >= 3U) { return false; }
    for (uint8_t i = 0; i < device_count_; ++i) {
        if (devices_[i]->config_.bus == config.bus) {
            // 控制 ID 固定 0x61，同一路总线只注册一块超电板。
            return false;
        }
    }
    if (!observer_registered_) {
        if (!bspFdcanAddObserver(onCanFrame)) { return false; }
        observer_registered_ = true;
    }
    config_ = config;
    initialized_ = true;
    devices_[device_count_++] = this;
    return true;
}

void Supercap::onCanFrame(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
                         const uint8_t* data, uint8_t length)
{
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME
        || header->FDFormat != FDCAN_CLASSIC_CAN) { return; }
    for (uint8_t i = 0; i < device_count_; ++i) {
        if (devices_[i]->config_.bus == bus) {
            (void)devices_[i]->update(header->Identifier, data, length);
        }
    }
}

bool Supercap::update(uint32_t identifier, const uint8_t* data, uint8_t length)
{
    if (!initialized_ || identifier != config_.rx_identifier || data == nullptr || length != 8U) {
        return false;
    }
    SupercapFeedback parsed;
    parsed.status_code = data[0];
    parsed.chassis_power_w = ((float)readLe16(data + 1) - 16384.0f) / 64.0f;
    parsed.referee_power_w = ((float)readLe16(data + 3) - 16384.0f) / 64.0f;
    parsed.chassis_power_limit_w = (float)readLe16(data + 5);
    parsed.cap_energy_raw = data[7];
    IrqLock lock;
    feedback_.data = parsed;
    feedback_.last_rx_tick = HAL_GetTick();
    ++feedback_.rx_count;
    received_ = true;
    return true;
}

SupercapSnapshot Supercap::readSnapshot() const
{
    IrqLock lock;
    SupercapSnapshot result = feedback_;
    result.online = received_
        && (HAL_GetTick() - result.last_rx_tick) <= config_.offline_timeout_ms;
    return result;
}

void Supercap::resetFeedback()
{
    IrqLock lock;
    feedback_ = SupercapSnapshot{};
    received_ = false;
}

bool Supercap::updateReferee(float power_w, float buffer_j)
{
    IrqLock lock;
    if (!initialized_) { return false; }
    if (validInput(power_w, config_.max_referee_power_w)) {
        power_w_ = (uint16_t)power_w;
        power_valid_ = true;
    }
    if (validInput(buffer_j, config_.max_buffer_j)) {
        buffer_j_ = (uint16_t)buffer_j;
        buffer_valid_ = true;
    }
    return power_valid_ && buffer_valid_;
}

bool Supercap::sendControl(const SupercapControl& control)
{
    uint8_t data[8] = {};
    {
        IrqLock lock;
        if (!initialized_ || !power_valid_ || !buffer_valid_) { return false; }
        // 显式按位、小端打包，避免 packed 位域和编译器布局依赖。
        data[0] = (control.enable_dcdc ? 0x01U : 0U)
            | (control.restart ? 0x02U : 0U) | (control.clear_error ? 0x20U : 0U)
            | (control.active_charging_limit ? 0x40U : 0U) | 0x80U;
        data[1] = (uint8_t)power_w_;
        data[2] = (uint8_t)(power_w_ >> 8);
        data[3] = (uint8_t)buffer_j_;
        data[4] = (uint8_t)(buffer_j_ >> 8);
        data[5] = control.charging_limit_ratio;
    }
    return bspFdcanSend(config_.bus, 0x61U, data, 8U);
}

bool Supercap::chassisPowerLimit(float* watts) const
{
    if (watts == nullptr) { return false; }
    const auto snapshot = readSnapshot();
    *watts = 0.0f;
    if (!snapshot.online || snapshot.data.chassis_power_limit_w <= 0.0f) { return false; }
    *watts = snapshot.data.chassis_power_limit_w;
    return true;
}
