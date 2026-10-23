#include "app_supercap.h"
#include "supercap_config.h"
#include "dvc_supercap.hpp"
#include "referee.hpp"
#include "cmsis_os.h"

namespace {
class IrqLock {
public:
    IrqLock() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(mask_); }
private:
    uint32_t mask_;
};
Supercap capacitor;
SupercapAppSnapshot state{};
bool initialized = false;
// 与提供的 superpower_task.c 一致，默认请求使能 DCDC。
bool enabled_request = true;
}

void superpower_task(void const* argument)
{
    (void)argument;
    for (;;) {
        const auto referee_data = referee.GetPowerSupplySnapshot();
        const bool values_ready = capacitor.updateReferee(
            referee_data.power_received ? (float)referee_data.power_limit_w : 0.0f,
            referee_data.buffer_received ? (float)referee_data.buffer_energy_j : 0.0f);
        const uint32_t now = HAL_GetTick();
        const bool fresh = referee_data.online && referee_data.power_received
            && referee_data.buffer_received
            && now - referee_data.power_tick <= SUPERCAP_REFEREE_TIMEOUT_MS
            && now - referee_data.buffer_tick <= SUPERCAP_REFEREE_TIMEOUT_MS;
        const auto feedback = capacitor.readSnapshot();
        SupercapControl command;
        bool requested;
        { IrqLock lock; requested = enabled_request; }
        // 先发送使能再等待反馈，避免需要控制帧才回报的超电板无法启动。
        command.enable_dcdc = requested && values_ready && fresh;
        command.charging_limit_ratio = SUPERCAP_CHARGING_LIMIT_RATIO;
        // 无首帧有效裁判值时不发送；离线后沿用有效值并发送失能。
        const bool queued = values_ready && capacitor.sendControl(command);
        {
            IrqLock lock;
            if (values_ready) {
                if (queued) { ++state.tx_ok; } else { ++state.tx_failed; }
            }
            state.status = !values_ready ? SUPERCAP_WAIT_REFEREE
                : !fresh ? SUPERCAP_REFEREE_OFFLINE
                : !feedback.online ? SUPERCAP_WAIT_FEEDBACK : SUPERCAP_READY;
            state.online = feedback.online;
            state.rx_count = feedback.rx_count;
            state.enable_requested = requested;
            state.enable_sent = queued && command.enable_dcdc;
            state.chassis_power_w = feedback.data.chassis_power_w;
            state.referee_power_w = feedback.data.referee_power_w;
            state.chassis_power_limit_w = feedback.data.chassis_power_limit_w;
            state.cap_energy_raw = feedback.data.cap_energy_raw;
            state.status_code = feedback.data.status_code;
        }
        osDelay(SUPERCAP_TX_PERIOD_MS);
    }
}

bool supercapInit(void)
{
    if (initialized) { return true; }
    if (SUPERCAP_FEEDBACK_CAN_ID > 0x7FFU) { return false; }
    SupercapConfig config;
    config.bus = bspFdcanGetHandle(SUPERCAP_CAN_INDEX);
    config.rx_identifier = SUPERCAP_FEEDBACK_CAN_ID;
    config.offline_timeout_ms = SUPERCAP_FEEDBACK_TIMEOUT_MS;
    config.max_referee_power_w = SUPERCAP_MAX_REFEREE_POWER_W;
    config.max_buffer_j = SUPERCAP_MAX_BUFFER_J;
    if (!capacitor.init(config)) { state.status = SUPERCAP_INIT_FAILED; return false; }
    state.status = SUPERCAP_WAIT_REFEREE;
    initialized = true;
    return true;
}

void supercapSetEnabled(bool enabled)
{
    IrqLock lock;
    enabled_request = enabled;
}

SupercapAppSnapshot supercapReadSnapshot(void)
{
    IrqLock lock;
    SupercapAppSnapshot result = state;
    result.enable_requested = enabled_request;
    result.online = capacitor.readSnapshot().online;
    return result;
}

bool supercapGetChassisPowerLimit(float* watts)
{
    if (watts == nullptr) { return false; }
    *watts = 0.0f;
    const auto snapshot = supercapReadSnapshot();
    if (!snapshot.online || snapshot.status != SUPERCAP_READY) { return false; }
    return capacitor.chassisPowerLimit(watts);
}
