/**
 * @file dvc_dm_motor.cpp
 * @author LuojiaFox
 * @brief 达妙电机驱动实现
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dm_motor.hpp"

#include <stddef.h>

/* DmMotor -------------------------------------------------------------------*/

DmMotor::DmMotor()
    : MotorBase(MotorType::Dm, nullptr, 0U, 0U), tx_identifier_(0U), control_mode_(DmControlMode::Mit)
{
}

bool DmMotor::init(
    uint8_t id,
    FDCAN_HandleTypeDef* bus,
    DmControlMode mode,
    const DmParamRange& range,
    uint32_t tx_identifier,
    uint32_t rx_identifier,
    int8_t direction,
    float reduction_ratio
)
{
    if (!isfinite(reduction_ratio) || reduction_ratio < 1.0e-6f || reduction_ratio > 1.0e6f
        || isRegistered() || bus == nullptr || id == 0U || tx_identifier == 0U
        || rx_identifier > 0x7FFU || tx_identifier > 0x7FFU - (uint32_t)mode
        || (mode != DmControlMode::Mit && mode != DmControlMode::PositionVelocity
            && mode != DmControlMode::Velocity)
        || !isfinite(range.position_max) || !isfinite(range.velocity_max)
        || !isfinite(range.torque_max) || range.position_max <= 0.0f
        || range.velocity_max <= 0.0f || range.torque_max <= 0.0f
        || range.position_max > 1.0e6f || range.velocity_max > 1.0e6f
        || range.torque_max > 1.0e6f) {
        return false;
    }

    configure(MotorType::Dm, bus, id, rx_identifier);
    tx_identifier_ = tx_identifier;
    control_mode_ = mode;
    range_ = range;
    setDirection(direction);
    setReductionRatio(reduction_ratio);
    reset();

    return true;
}

bool DmMotor::update(uint32_t identifier, const uint8_t* data, uint8_t length)
{
    if ((length != 8U) || (data == nullptr) || (bus() == nullptr) || (identifier != rxIdentifier())) {
        return false;
    }

    const uint16_t position_int =
        (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[2]);
    const uint16_t velocity_int =
        (uint16_t)(((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));
    const uint16_t torque_int =
        (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | (uint16_t)data[5]);

    data_.state = (uint8_t)(data[0] >> 4);
    data_.position_rad = uintToFloat(position_int, -range_.position_max, range_.position_max, 16U);
    data_.velocity_rad_s =
        uintToFloat(velocity_int, -range_.velocity_max, range_.velocity_max, 12U);
    data_.torque_nm = uintToFloat(torque_int, -range_.torque_max, range_.torque_max, 12U);
    data_.position_rad *= (float)direction() / reductionRatio();
    data_.velocity_rad_s *= (float)direction() / reductionRatio();
    data_.torque_nm *= (float)direction() * reductionRatio();
    data_.temperature_mos = (float)data[6];
    data_.temperature_rotor = (float)data[7];
    data_.initialized = true;

    refreshRxTick();
    return true;
}

void DmMotor::reset()
{
    MotorCriticalSection lock;
    data_ = DmMotorData{};
    clearRxTick();
}

bool DmMotor::sendCommand(DmCommand command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U};

    switch (command) {
        case DmCommand::Enable:
            data[7] = 0xFCU;
            break;

        case DmCommand::Disable:
            data[7] = 0xFDU;
            break;

        case DmCommand::SaveZeroPosition:
            data[7] = 0xFEU;
            break;

        default:
            return false;
    }

    return bspFdcanSend(bus(), tx_identifier_ + modeOffset(), data, 8U);
}

bool DmMotor::sendMit(const DmControl& output)
{
    if (control_mode_ != DmControlMode::Mit || !isfinite(output.position_rad)
        || !isfinite(output.velocity_rad_s) || !isfinite(output.torque_nm)
        || !isfinite(output.kp) || !isfinite(output.kd)) { return false; }
    DmControl control = output;
    control.position_rad *= (float)direction() * reductionRatio();
    control.velocity_rad_s *= (float)direction() * reductionRatio();
    control.torque_nm *= (float)direction() / reductionRatio();
    control.kp /= reductionRatio() * reductionRatio();
    control.kd /= reductionRatio() * reductionRatio();
    if (!isfinite(control.position_rad) || !isfinite(control.velocity_rad_s)
        || !isfinite(control.torque_nm) || !isfinite(control.kp)
        || !isfinite(control.kd)) { return false; }
    const uint16_t position_int = floatToUint(
        control.position_rad,
        -range_.position_max,
        range_.position_max,
        16U
    );
    const uint16_t velocity_int = floatToUint(
        control.velocity_rad_s,
        -range_.velocity_max,
        range_.velocity_max,
        12U
    );
    const uint16_t torque_int = floatToUint(
        control.torque_nm,
        -range_.torque_max,
        range_.torque_max,
        12U
    );
    const uint16_t kp_int = floatToUint(control.kp, 0.0f, 500.0f, 12U);
    const uint16_t kd_int = floatToUint(control.kd, 0.0f, 5.0f, 12U);

    uint8_t data[8];
    data[0] = (uint8_t)(position_int >> 8);
    data[1] = (uint8_t)(position_int);
    data[2] = (uint8_t)(velocity_int >> 4);
    data[3] = (uint8_t)(((velocity_int & 0x0FU) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)(kp_int);
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0FU) << 4) | (torque_int >> 8));
    data[7] = (uint8_t)(torque_int);

    /* MIT 模式的命令标识符就是电机 ID */
    return bspFdcanSend(bus(), tx_identifier_, data, 8U);
}

bool DmMotor::sendPositionVelocity(float position_rad, float velocity_rad_s)
{
    if (control_mode_ != DmControlMode::PositionVelocity) { return false; }
    position_rad *= (float)direction() * reductionRatio();
    velocity_rad_s *= reductionRatio(); // 位置速度模式的速度字段为非负速度上限
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || velocity_rad_s < 0.0f
        || fabsf(position_rad) > range_.position_max || velocity_rad_s > range_.velocity_max) {
        return false;
    }
    const uint8_t* position_bytes = reinterpret_cast<const uint8_t*>(&position_rad);
    const uint8_t* velocity_bytes = reinterpret_cast<const uint8_t*>(&velocity_rad_s);

    uint8_t data[8];
    data[0] = position_bytes[0];
    data[1] = position_bytes[1];
    data[2] = position_bytes[2];
    data[3] = position_bytes[3];
    data[4] = velocity_bytes[0];
    data[5] = velocity_bytes[1];
    data[6] = velocity_bytes[2];
    data[7] = velocity_bytes[3];

    /* 位置-速度模式的命令标识符固定为 ID + 0x100 */
    return bspFdcanSend(
        bus(),
        tx_identifier_ + (uint32_t)DmControlMode::PositionVelocity,
        data,
        8U
    );
}

bool DmMotor::sendVelocity(float velocity_rad_s)
{
    if (control_mode_ != DmControlMode::Velocity) { return false; }
    velocity_rad_s *= (float)direction() * reductionRatio();
    if (!isfinite(velocity_rad_s) || fabsf(velocity_rad_s) > range_.velocity_max) {
        return false;
    }
    const uint8_t* velocity_bytes = reinterpret_cast<const uint8_t*>(&velocity_rad_s);

    uint8_t data[4];
    data[0] = velocity_bytes[0];
    data[1] = velocity_bytes[1];
    data[2] = velocity_bytes[2];
    data[3] = velocity_bytes[3];

    /* 速度模式的命令标识符固定为 ID + 0x200 */
    return bspFdcanSend(bus(), tx_identifier_ + (uint32_t)DmControlMode::Velocity, data, 4U);
}

uint32_t DmMotor::modeOffset() const
{
    return (uint32_t)control_mode_;
}

float DmMotor::uintToFloat(uint16_t value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_int = (1UL << bits) - 1UL;

    return (float)value * span / (float)max_int + min_value;
}

uint16_t DmMotor::floatToUint(float value, float min_value, float max_value, uint8_t bits)
{
    const float span = max_value - min_value;
    const uint32_t max_int = (1UL << bits) - 1UL;

    if (value < min_value) {
        value = min_value;
    } else if (value > max_value) {
        value = max_value;
    }

    return (uint16_t)((value - min_value) * (float)max_int / span);
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
