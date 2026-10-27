/**
 * @file dvc_motor.cpp
 * @author LuojiaFox
 * @brief 电机公共基类与 CAN 接收分发实现
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.hpp"

#include <stddef.h>

/* MotorBase static registry --------------------------------------------------*/

constexpr uint8_t MotorBase::kMaxMotors;

MotorBase* MotorBase::motors_[MotorBase::kMaxMotors] = {};
volatile uint8_t MotorBase::motor_count_ = 0U;

/* MotorBase -----------------------------------------------------------------*/

MotorBase::MotorBase(MotorType type, FDCAN_HandleTypeDef* bus, uint8_t id, uint32_t rx_identifier)
    : type_(type),
      bus_(bus),
      id_(id),
      rx_identifier_(rx_identifier),
      direction_(1),
      reduction_ratio_(1.0f),
      last_rx_tick_(0),
      rx_count_(0),
      offline_timeout_ms_(kDefaultMotorOfflineTimeoutMs)
{
}

void MotorBase::configure(
    MotorType type,
    FDCAN_HandleTypeDef* bus,
    uint8_t id,
    uint32_t rx_identifier
)
{
    type_ = type;
    bus_ = bus;
    id_ = id;
    rx_identifier_ = rx_identifier;
}

void MotorBase::refreshRxTick()
{
    received_ = true;
    last_rx_tick_ = HAL_GetTick();
    rx_count_++;
}

bool MotorBase::isOnline(uint32_t now_ms) const
{
    MotorCriticalSection lock;
    if (!received_) {
        return false;
    }
    return (now_ms - last_rx_tick_) <= offline_timeout_ms_;
}

bool MotorBase::isOnline() const
{
    MotorCriticalSection lock;
    return isOnline(HAL_GetTick());
}

bool MotorBase::isRegistered() const
{
    for (uint8_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] == this) { return true; }
    }
    return false;
}

bool MotorBase::registerMotor(MotorBase& motor)
{
    MotorCriticalSection lock;
    if (motor.bus() == nullptr || motor.id() == 0U || motor.type() == MotorType::Unknown
        || motor.rxIdentifier() > 0x7FFU || motor.txIdentifier() > 0x7FFU) {
        return false;
    }
    if (motor_count_ >= kMaxMotors) {
        return false;
    }

    for (uint8_t i = 0U; i < motor_count_; i++) {
        if (motors_[i] == &motor) {
            return false;
        }

        /* 同一路 CAN 上不能有两台电机共用同一个反馈标识符 */
        if ((motors_[i]->bus() == motor.bus())
            && (motors_[i]->rxIdentifier() == motor.rxIdentifier()))
        {
            return false;
        }

        /* 同一路 CAN 的同一发送帧、同一槽位只能对应一台电机 */
        if ((motor.txIdentifier() != 0U) && (motors_[i]->bus() == motor.bus())
            && (motors_[i]->txIdentifier() == motor.txIdentifier())
            && (!motor.sharesTxFrame() || !motors_[i]->sharesTxFrame()
                || (motors_[i]->txSlot() == motor.txSlot())))
        {
            return false;
        }

        if (motors_[i]->bus() == motor.bus()
            && ((motor.txIdentifier() != 0U && motor.txIdentifier() == motors_[i]->rxIdentifier())
                || (motors_[i]->txIdentifier() != 0U
                    && motors_[i]->txIdentifier() == motor.rxIdentifier()))) {
            return false;
        }
    }

    motors_[motor_count_] = &motor;
    motor_count_++;

    return true;
}

void MotorBase::onCanFrame(
    FDCAN_HandleTypeDef* bus,
    uint32_t identifier,
    const uint8_t* data,
    uint8_t length
)
{
    if ((bus == nullptr) || (data == nullptr)) {
        return;
    }

    for (uint8_t i = 0U; i < motor_count_; i++) {
        MotorBase* motor = motors_[i];
        if ((motor->bus() == bus) && (motor->rxIdentifier() == identifier)) {
            (void)motor->update(identifier, data, length);
            return;
        }
    }
}

MotorBase* MotorBase::motorAt(uint8_t index)
{
    if (index >= motor_count_) {
        return nullptr;
    }
    return motors_[index];
}

uint8_t MotorBase::motorCount()
{
    return motor_count_;
}

bool MotorBase::allOnline(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < motor_count_; i++) {
        if (!motors_[i]->isOnline(now_ms)) {
            return false;
        }
    }
    return true;
}

/* CAN RX bridge -------------------------------------------------------------*/

extern "C" void bspFdcanRxHandler(
    FDCAN_HandleTypeDef* hcan,
    uint32_t identifier,
    const uint8_t* data,
    uint8_t length
)
{
    MotorBase::onCanFrame(hcan, identifier, data, length);
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
