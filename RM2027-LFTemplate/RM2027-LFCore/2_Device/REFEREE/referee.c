/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note       裁判系统通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.h"

#include "CRC8_CRC16.h"
#include "protocol.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "string.h"

#define REFEREE_TIMEOUT 300  //裁判系统超时时间
uint32_t referee_online_time = 0;
uint32_t referee_receive_count = 0;

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_status;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
referee_warning_t referee_warning;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t game_robot_pos_t;
buff_t buff_musk_t;
air_support_data_t robot_energy_t;
hurt_data_t robot_hurt_t;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
robot_interaction_data_t student_interactive_data_t;
CustomControllerData_t CUSTOM_CONTROLLER_DATA;  //自定义控制器数据
MapCommandData_t MAP_COMMAND_DATA;  //地图命令数据


rfid_status_t RFID_STATUS;
ground_robot_position_t ground_robot_pos_t;
sentry_info_t sentry_decision_info_t;

ext_robot_command_t robot_command_t;


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));

    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));

    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));
    memset(&CUSTOM_CONTROLLER_DATA, 0, sizeof(CustomControllerData_t));
    memset(&MAP_COMMAND_DATA, 0, sizeof(MapCommandData_t));
    memset(&robot_command_t, 0, sizeof(ext_robot_command_t));
}

void referee_data_solve(uint8_t * frame)
{
    if (HAL_GetTick() - referee_online_time > REFEREE_TIMEOUT) {
        referee_receive_count = 0;
    }
    referee_receive_count++;

    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATE_CMD_ID: {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            referee_online_time = HAL_GetTick();
        } break;

        case FIELD_EVENTS_CMD_ID: {
            memcpy(&field_event, frame + index, sizeof(event_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
            memcpy(
                &supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID: {
            memcpy(
                &supply_projectile_booking_t, frame + index,
                sizeof(ext_supply_projectile_booking_t));
            referee_online_time = HAL_GetTick();
        } break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            referee_online_time = HAL_GetTick();
        } break;

        case ROBOT_STATE_CMD_ID: {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case POWER_HEAT_DATA_CMD_ID: {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&game_robot_pos_t, frame + index, sizeof(robot_pos_t));
            referee_online_time = HAL_GetTick();
        } break;
        case BUFF_MUSK_CMD_ID: {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_t));
            referee_online_time = HAL_GetTick();
        } break;
        case AERIAL_ROBOT_ENERGY_CMD_ID: {
            memcpy(&robot_energy_t, frame + index, sizeof(air_support_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_HURT_CMD_ID: {
            memcpy(&robot_hurt_t, frame + index, sizeof(hurt_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case PROJECTILE_ALLOWANCE_CMD_ID: {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            referee_online_time = HAL_GetTick();
        } break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID: {
            memcpy(&student_interactive_data_t, frame + index, sizeof(robot_interaction_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case CUSTOM_CONTROLLER_CMD_ID: {
            memcpy(&CUSTOM_CONTROLLER_DATA, frame + index, sizeof(CustomControllerData_t));
            referee_online_time = HAL_GetTick();
        } break;
        case MAP_COMMAND_CMD_ID: {
            memcpy(&MAP_COMMAND_DATA, frame + index, sizeof(MapCommandData_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_COMMAND_CMD_ID: {
            memcpy(&robot_command_t, frame + index, sizeof(ext_robot_command_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RFID_STATUS_ID: {
            memcpy(&RFID_STATUS, frame + index, sizeof(rfid_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GOUND_ROBOT_POS_ID: {
            memcpy(&ground_robot_pos_t, frame + index, sizeof(robot_pos_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SENTRY_DECISION_INFO_ID: {
            memcpy(&sentry_decision_info_t, frame + index, sizeof(sentry_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        default: {
            referee_receive_count--;
            break;
        }
    }
}

void get_chassis_power_and_buffer(float * power, float * buffer)
{
    *power = power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;
}

float get_chassis_power(void) { return power_heat_data.chassis_power; }
float get_buffer(void) { return power_heat_data.buffer_energy; }
void get_chassis_power_limit(uint16_t * power_limit) { *power_limit = robot_status.chassis_power_limit; }



uint8_t get_robot_id(void) { return robot_status.robot_id; }

uint16_t get_shoot_heat(void)
{
    return power_heat_data.shooter_17mm_barrel_heat;
}

void get_shoot_heat17_limit_and_heat17(uint16_t * cooling_value, uint16_t * heat_limit, uint16_t * heat)
{
    *cooling_value = robot_status.shooter_barrel_cooling_value;
    *heat_limit = robot_status.shooter_barrel_heat_limit;
    *heat = power_heat_data.shooter_17mm_barrel_heat;
}

// void get_shoot_heat42_limit_and_heat42(uint16_t * heat_limit, uint16_t * heat)
// {
//     *heat_limit = robot_status.shooter_barrel_heat_limit;
//     *heat = power_heat_data.shooter_42mm_barrel_heat;
// }

void get_shoot_speed_fre(uint8_t *shoot_fre , float *shoot_speed)
{
    *shoot_fre = shoot_data.launching_frequency;
    *shoot_speed = shoot_data.initial_speed;
}
/**
 * @brief 反馈机器人颜色
 * @param  none
 * @return 0为红色,1为蓝色,2为未知
 */
uint8_t get_team_color(void)  // 谨防“哨兵在打我”
{
    switch (robot_status.robot_id) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            return 0;
        case 101:
        case 102:
        case 103:
        case 104:
        case 105:
        case 106:
        case 107:
        case 108:
        case 109:
        case 110:
        case 111:
            return 1;
        default:
            return 2;
    }
}

CustomControllerData_t * GetCustomControllerDataPoint(void) { return &CUSTOM_CONTROLLER_DATA; }

void GetMapCommandData(float * x, float * y, uint8_t * cmd_keyboard, uint8_t * target_robot_id, uint16_t * cmd_source)
{
    *x = MAP_COMMAND_DATA.target_position_x;
    *y = MAP_COMMAND_DATA.target_position_y;
    *cmd_keyboard = MAP_COMMAND_DATA.cmd_keyboard;
    *target_robot_id = MAP_COMMAND_DATA.target_robot_id;
    *cmd_source = MAP_COMMAND_DATA.cmd_source;
}

void GetSentryInfo(uint32_t * sentry_info, uint16_t * sentry_info_2)
{
    *sentry_info = sentry_decision_info_t.sentry_info;
    *sentry_info_2 = sentry_decision_info_t.sentry_info_2;
}
uint16_t get_current_HP(void)
{
    return robot_status.current_HP;
}

/*========== API ==========*/

inline bool GetRefereeOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    return !((referee_receive_count > 5) && (current_time - referee_online_time < REFEREE_TIMEOUT));
}

/**
 * @brief 获取自定义控制器数据
 * @param  index 数据索引
 * @return float 数据
 */
inline float GetCustomControllerPos(uint8_t index)
{
    float data = 0;
    memcpy(&data, &CUSTOM_CONTROLLER_DATA.data[index * 4], 4);
    return data;
    // return *((float *)(&CUSTOM_CONTROLLER_DATA.data[index * 4]));
}

/*------------------------------ End of File ------------------------------*/
