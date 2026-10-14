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

#include "LF.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "app_referee.h"

extern "C" {
LuojiaFox::SystemStateSnapshot g_system_state{};
}

namespace LuojiaFox {

SystemStateStore system_state;

void SystemStateStore::UpdateImu(const ImuState &state)
{
    taskENTER_CRITICAL();
    g_system_state.imu = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::UpdateGimbal(const GimbalState &state)
{
    taskENTER_CRITICAL();
    g_system_state.gimbal = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::UpdateShooter(const ShooterState &state)
{
    taskENTER_CRITICAL();
    g_system_state.shooter = state;
    taskEXIT_CRITICAL();
}

void SystemStateStore::UpdateReferee(const RefereeState &state)
{
    taskENTER_CRITICAL();
    g_system_state.referee = state;
    taskEXIT_CRITICAL();
}

SystemStateSnapshot SystemStateStore::Read() const
{
    taskENTER_CRITICAL();
    const SystemStateSnapshot result = g_system_state;
    taskEXIT_CRITICAL();
    return result;
}

} // namespace LuojiaFox

extern "C" void LF_UpdateReferee(uint16_t robot_id, uint16_t remaining_hp, uint8_t connected)
{
    LuojiaFox::RefereeState state;

    state.robot_id     = robot_id;
    state.remaining_hp = remaining_hp;
    state.connected    = (connected != 0U);

    LuojiaFox::system_state.UpdateReferee(state);
}