/**
 * @file LF.cpp
 * @author BayekQ (206858817@qq.com)
 * @brief LuojiaFox 驱动
 * @version 0.1
 * @date TODO
 *
 * @copyright WHU-LuojiaFox (c) 2026
 *
 */


#include "lf_core.hpp"

#include "FreeRTOS.h"
#include "task.h"

extern "C" {
LuojiaFox::SystemStateSnapshot g_system_state{};
}

namespace LuojiaFox {

SystemStateStore system_state;

void SystemStateStore::updateImu(const ImuState &state)
{
    taskENTER_CRITICAL();
    g_system_state.imu = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateGimbal(const GimbalState &state)
{
    taskENTER_CRITICAL();
    g_system_state.gimbal = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateShooter(const ShooterState &state)
{
    taskENTER_CRITICAL();
    g_system_state.shooter = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::updateReferee(const RefereeState &state)
{
    taskENTER_CRITICAL();
    g_system_state.referee = state;
    taskEXIT_CRITICAL();
}

SystemStateSnapshot SystemStateStore::read() const
{
    taskENTER_CRITICAL();
    const SystemStateSnapshot result = g_system_state;
    taskEXIT_CRITICAL();
    return result;
}

} // namespace LuojiaFox