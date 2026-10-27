/**
 * @file dvc_lk_motor.cpp
 * @author LuojiaFox
 * @brief 瓴控电机驱动实现
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_lk_motor.hpp"

#include <stddef.h>

/* Private macros ------------------------------------------------------------*/

namespace {

/** @brief 角度单位换算 */
constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717958f;
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToCentiDegree = 5729.57795131f; /* rad -> 0.01 deg */

/** @brief 瓴控电机命令标识符基址 */
constexpr uint32_t kLkCommandBase = 0x140U;

/** @brief 瓴控电机电流比例 */
constexpr float kLkCurrentPerLsbMf = 33.0f / 4096.0f;
constexpr float kLkCurrentPerLsbMg = 66.0f / 4096.0f;

/**
 * @brief 判断是否为状态反馈帧
 * @note  0x9C 是显式读状态响应；部分驱动器会回显控制命令字
 *        （0xA1 转矩 / 0xA2 速度 / 0xA3、0xA4 位置），数据布局相同。
 */
bool isFeedbackFrame(uint8_t command)
{
    switch (command) {
        case 0x9CU:
        case 0xA1U:
        case 0xA2U:
        case 0xA3U:
        case 0xA4U:
            return true;

        default:
            return false;
    }
}

}  // namespace

/* LkMotor -------------------------------------------------------------------*/

LkMotor::LkMotor()
    : MotorBase(MotorType::Lk, nullptr, 0U, 0U),
      encoder_bits_(16U),
      current_per_lsb_(kLkCurrentPerLsbMf)
{
}

bool LkMotor::init(
    uint8_t id,
    FDCAN_HandleTypeDef* bus,
    uint8_t encoder_bits,
    bool mg_current,
    float reduction_ratio,
    int8_t direction
)
{
    if (isRegistered() || !isfinite(reduction_ratio) || (reduction_ratio < 1.0e-6f || reduction_ratio > 1.0e6f)
        || (bus == nullptr) || (id == 0U) || (id > 32U) || (encoder_bits < 14U)
        || (encoder_bits > 16U))
    {
        return false;
    }

    configure(MotorType::Lk, bus, id, kLkCommandBase + (uint32_t)id);
    encoder_bits_ = encoder_bits;
    current_per_lsb_ = mg_current ? kLkCurrentPerLsbMg : kLkCurrentPerLsbMf;
    setReductionRatio(reduction_ratio);
    setDirection(direction);
    reset();

    return true;
}

bool LkMotor::update(uint32_t identifier, const uint8_t* data, uint8_t length)
{
    if ((length != 8U) || (data == nullptr) || (bus() == nullptr)
        || (identifier != rxIdentifier()) || !isFeedbackFrame(data[0])) {
        return false;
    }

    const uint16_t encoder = (uint16_t)(((uint16_t)data[7] << 8) | data[6]);
    if (encoder >= (1UL << encoder_bits_)) { return false; }
    data_.frame_type = data[0];
    data_.temperature = (int8_t)data[1];
    data_.iq_raw = (int16_t)(((uint16_t)data[3] << 8) | (uint16_t)data[2]);
    data_.speed_dps = (int16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[4]);
    data_.encoder = (uint16_t)(((uint16_t)data[7] << 8) | (uint16_t)data[6]);

    data_.current_a = (float)data_.iq_raw * current_per_lsb_ * (float)direction();
    data_.velocity_rad_s =
        (float)data_.speed_dps * kDegToRad / reductionRatio() * (float)direction();

    const uint32_t encoder_max = 1UL << encoder_bits_;
    data_.angle_rad = (float)data_.encoder * kTwoPi / (float)encoder_max;
    if (data_.angle_rad > kPi) { data_.angle_rad -= kTwoPi; }
    data_.angle_rad *= (float)direction() / reductionRatio();

    for (uint8_t i = 0U; i < 8U; i++) {
        data_.raw[i] = data[i];
    }

    data_.initialized = true;
    refreshRxTick();
    return true;
}

void LkMotor::reset()
{
    MotorCriticalSection lock;
    data_ = LkMotorData{};
    clearRxTick();
}

bool LkMotor::sendCommand(
    uint8_t command,
    uint8_t offset,
    const uint8_t* payload,
    uint8_t length
)
{
    if ((bus() == nullptr) || (offset > 8U) || (length > (uint8_t)(8U - offset))
        || ((length > 0U) && (payload == nullptr)))
    {
        return false;
    }

    uint8_t data[8] = {};
    data[0] = command;
    for (uint8_t i = 0U; i < length; i++) {
        data[offset + i] = payload[i];
    }

    return bspFdcanSend(bus(), commandIdentifier(), data, 8U);
}

bool LkMotor::sendEnable()
{
    return sendCommand(0x88U, 1U, nullptr, 0U);
}

bool LkMotor::sendDisable()
{
    return sendCommand(0x80U, 1U, nullptr, 0U);
}

bool LkMotor::sendStop()
{
    return sendCommand(0x81U, 1U, nullptr, 0U);
}

bool LkMotor::sendReadStatus()
{
    return sendCommand(0x9CU, 1U, nullptr, 0U);
}

int16_t LkMotor::clampIq(int32_t value)
{
    if (value > 2048) {
        value = 2048;
    } else if (value < -2048) {
        value = -2048;
    }
    return (int16_t)value;
}

bool LkMotor::sendTorqueCurrent(int16_t iq)
{
    const int16_t value = (int16_t)(clampIq(iq) * direction());
    uint8_t payload[2];
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)((uint16_t)value >> 8);

    return sendCommand(0xA1U, 4U, payload, 2U);
}

bool LkMotor::sendCurrent(float current_a)
{
    if (!isfinite(current_a) || current_per_lsb_ <= 0.0f) {
        return false;
    }

    float raw = current_a / current_per_lsb_;
    if (raw > 2048.0f) { raw = 2048.0f; }
    if (raw < -2048.0f) { raw = -2048.0f; }
    return sendTorqueCurrent((int16_t)raw);
}

bool LkMotor::sendSpeed(float velocity_rad_s, float current_limit_a)
{
    if (!isfinite(current_limit_a)) { return false; }
    const float raw_limit = fminf(fabsf(current_limit_a) / current_per_lsb_, 2048.0f);
    if (current_limit_a != 0.0f && raw_limit < 1.0f) { return false; }
    const int16_t limit = (int16_t)raw_limit;
    const double raw_speed = (double)velocity_rad_s * (double)kRadToCentiDegree
                             * (double)reductionRatio() * direction();
    if (!isfinite(raw_speed) || raw_speed > INT32_MAX || raw_speed < INT32_MIN) { return false; }
    const int32_t speed = (int32_t)raw_speed;

    uint8_t payload[6];
    payload[0] = (uint8_t)limit;
    payload[1] = (uint8_t)((uint16_t)limit >> 8);
    payload[2] = (uint8_t)((uint32_t)speed);
    payload[3] = (uint8_t)((uint32_t)speed >> 8);
    payload[4] = (uint8_t)((uint32_t)speed >> 16);
    payload[5] = (uint8_t)((uint32_t)speed >> 24);

    return sendCommand(0xA2U, 2U, payload, 6U);
}

bool LkMotor::sendPosition(float angle_rad, float max_velocity_rad_s)
{
    if (!isfinite(max_velocity_rad_s) || max_velocity_rad_s < 0.0f) { return false; }
    const double raw_angle = (double)angle_rad * (double)kRadToCentiDegree
                             * (double)reductionRatio() * direction();
    if (!isfinite(raw_angle) || raw_angle > INT32_MAX || raw_angle < INT32_MIN) { return false; }
    const int32_t angle = (int32_t)raw_angle;

    uint8_t angle_bytes[4];
    angle_bytes[0] = (uint8_t)((uint32_t)angle);
    angle_bytes[1] = (uint8_t)((uint32_t)angle >> 8);
    angle_bytes[2] = (uint8_t)((uint32_t)angle >> 16);
    angle_bytes[3] = (uint8_t)((uint32_t)angle >> 24);

    if (max_velocity_rad_s == 0.0f) {
        return sendCommand(0xA3U, 4U, angle_bytes, 4U);
    }

    const double speed_dps = (double)max_velocity_rad_s / (double)kDegToRad * (double)reductionRatio();
    // 0xA4 的速度字段为 uint16_t，360 是单位示例，不是协议上限。
    if (speed_dps < 1.0 || speed_dps > UINT16_MAX) { return false; }
    const uint16_t speed = (uint16_t)speed_dps;

    uint8_t payload[6];
    payload[0] = (uint8_t)speed;
    payload[1] = (uint8_t)(speed >> 8);
    payload[2] = angle_bytes[0];
    payload[3] = angle_bytes[1];
    payload[4] = angle_bytes[2];
    payload[5] = angle_bytes[3];

    return sendCommand(0xA4U, 2U, payload, 6U);
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
