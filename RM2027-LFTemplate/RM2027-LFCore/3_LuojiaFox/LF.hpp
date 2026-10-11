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

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

#ifdef __cplusplus

extern "C++" {
namespace LuojiaFox {

struct ImuState
{
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float yaw_rate_dps;
    float pitch_rate_dps;
    float roll_rate_dps;
};

struct GimbalState
{
    float yaw_deg;
    float pitch_deg;
    bool online;
};

struct ShooterState
{
    uint16_t friction_wheel_rpm;
    uint16_t ammunition_remaining;
    bool ready;
};

struct RefereeState
{
    uint16_t robot_id;
    uint16_t remaining_hp;
    bool connected;
};

struct SystemStateSnapshot
{
    ImuState imu;
    GimbalState gimbal;
    ShooterState shooter;
    RefereeState referee;
};

class SystemStateStore
{
public:
    // Call these methods from FreeRTOS task context, not from an ISR.
    void UpdateImu(const ImuState &state);
    void UpdateGimbal(const GimbalState &state);
    void UpdateShooter(const ShooterState &state);
    void UpdateReferee(const RefereeState &state);
    SystemStateSnapshot Read() const;
};

extern SystemStateStore system_state;

} // namespace LuojiaFox
}

extern "C" {
extern LuojiaFox::SystemStateSnapshot g_system_state;
}

#endif
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
