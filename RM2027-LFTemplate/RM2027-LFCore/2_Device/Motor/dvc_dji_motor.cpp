/**
 * @file dvc_dji_motor.cpp
 * @author LuojiaFox
 * @brief DJI 电机驱动实现
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dji_motor.hpp"

#include <stddef.h>

/* Private macros ------------------------------------------------------------*/

namespace {

/** @brief rpm -> rad/s，即 2*pi/60 */
constexpr float kRpmToRadPerSecond = 0.1047197551f;

/** @brief 把角度折算到 -180~180 度 */
float wrap180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/** @brief DJI 电机默认减速比 */
float defaultReductionRatio(MotorType type)
{
    switch (type) {
        case MotorType::DjiM3508:
            return 19.0f;
        case MotorType::DjiM2006:
            return 36.0f;
        case MotorType::DjiGm6020:
            return 1.0f;
        default:
            return 1.0f;
    }
}

}  // namespace

/* DjiMotor ------------------------------------------------------------------*/

DjiMotor::DjiMotor() : MotorBase(MotorType::Unknown, nullptr, 0U, 0U), command_(0)
{
}

bool DjiMotor::init(
    MotorType type,
    uint8_t id,
    FDCAN_HandleTypeDef* bus,
    int8_t direction,
    float reduction_ratio
)
{
    if (isRegistered() || bus == nullptr || !isfinite(reduction_ratio)
        || (reduction_ratio != 0.0f
            && (reduction_ratio < 1.0e-6f || reduction_ratio > 1.0e6f))) {
        return false;
    }

    uint8_t max_id = 0;
    uint32_t rx_identifier = 0;

    switch (type) {
        case MotorType::DjiGm6020: {
            max_id = 7;
            rx_identifier = 0x205U + (uint32_t)id - 1U;
        } break;

        case MotorType::DjiM3508:
        case MotorType::DjiM2006: {
            max_id = 8;
            rx_identifier = 0x201U + (uint32_t)id - 1U;
        } break;

        default:
            return false;
    }

    if ((id == 0U) || (id > max_id)) {
        return false;
    }

    configure(type, bus, id, rx_identifier);
    setDirection(direction);
    setReductionRatio((reduction_ratio > 0.0f) ? reduction_ratio : defaultReductionRatio(type));
    reset();

    return true;
}

bool DjiMotor::update(uint32_t identifier, const uint8_t* data, uint8_t length)
{
    if ((length != 8U) || (data == nullptr) || (bus() == nullptr) || (identifier != rxIdentifier())) {
        return false;
    }

    const uint16_t encoder =
        (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    if (encoder >= kDjiEncoderRange) { return false; }
    const int16_t velocity_rpm =
        (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    const int16_t current_raw =
        (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);

    if (!data_.initialized) {
        data_.last_encoder = encoder;
        data_.initialized = true;
    } else {
        int32_t delta = (int32_t)encoder - (int32_t)data_.last_encoder;

        /* 编码器在两个方向上都可能跨零点，按最短路径取增量 */
        if (delta > (int32_t)(kDjiEncoderRange / 2U)) {
            delta -= (int32_t)kDjiEncoderRange;
        } else if (delta < -(int32_t)(kDjiEncoderRange / 2U)) {
            delta += (int32_t)kDjiEncoderRange;
        }

        data_.last_encoder = encoder;
        data_.total_angle_deg += (float)delta * 360.0f
                                 / ((float)kDjiEncoderRange * reductionRatio())
                                 * (float)direction();
        data_.angle_deg = wrap180(data_.total_angle_deg);
    }

    data_.encoder = encoder;
    data_.velocity_rpm = velocity_rpm;
    data_.current_raw = current_raw;
    data_.temperature = data[6];
    data_.velocity_rad_s =
        (float)velocity_rpm * kRpmToRadPerSecond / reductionRatio() * (float)direction();

    for (uint8_t i = 0U; i < 8U; i++) {
        data_.raw[i] = data[i];
    }

    refreshRxTick();
    return true;
}

void DjiMotor::reset()
{
    MotorCriticalSection lock;
    data_ = DjiMotorData{};
    command_ = 0;
    clearRxTick();
}

uint32_t DjiMotor::txIdentifier() const
{
    switch (type()) {
        case MotorType::DjiGm6020:
            return (id() <= 4U) ? 0x1FFU : 0x2FFU;

        case MotorType::DjiM3508:
        case MotorType::DjiM2006:
            return (id() <= 4U) ? 0x200U : 0x1FFU;

        default:
            return 0U;
    }
}

uint8_t DjiMotor::txSlot() const
{
    return (uint8_t)((id() - 1U) % 4U);
}

void DjiMotor::setCommand(int16_t value)
{
    MotorCriticalSection lock;
    int32_t limit = 0;

    switch (type()) {
        case MotorType::DjiGm6020:
            limit = 30000;
            break;

        case MotorType::DjiM3508:
            limit = 16384;
            break;

        case MotorType::DjiM2006:
            limit = 10000;
            break;

        default:
            command_ = 0;
            return;
    }

    if (value > limit) {
        value = (int16_t)limit;
    } else if (value < -limit) {
        value = (int16_t)(-limit);
    }

    command_ = value;
}

bool DjiMotor::sendFrame(
    FDCAN_HandleTypeDef* bus,
    uint32_t identifier,
    const int16_t current[4]
)
{
    if ((bus == nullptr) || (current == nullptr)
        || (identifier != 0x200U && identifier != 0x1FFU && identifier != 0x2FFU)) {
        return false;
    }

    uint8_t data[8];
    for (uint8_t i = 0U; i < 4U; i++) {
        data[2U * i] = (uint8_t)((uint16_t)current[i] >> 8);
        data[2U * i + 1U] = (uint8_t)(current[i]);
    }

    return bspFdcanSend(bus, identifier, data, 8U);
}

bool DjiMotor::sendGroup(FDCAN_HandleTypeDef* bus, uint32_t identifier)
{
    if (bus == nullptr) { return false; }
    int16_t current[4] = {};
    bool matched = false;
    {
        MotorCriticalSection lock;
        for (uint8_t i = 0; i < motorCount(); ++i) {
            MotorBase* base = motorAt(i);
            if (base->bus() != bus || base->txIdentifier() != identifier
                || (base->type() != MotorType::DjiGm6020
                    && base->type() != MotorType::DjiM3508
                    && base->type() != MotorType::DjiM2006)) { continue; }
            DjiMotor* motor = static_cast<DjiMotor*>(base);
            current[motor->txSlot()] = static_cast<int16_t>(motor->command() * motor->direction());
            matched = true;
        }
    }
    return matched && sendFrame(bus, identifier, current);
}
