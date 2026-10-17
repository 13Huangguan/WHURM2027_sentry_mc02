#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include <stdint.h>

// clang-format off
#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    /* 取值见《RoboMaster 2026 通信协议 V2.0.0》表 1-5。
       0x0102 / 0x0103 / 0x0205 / 0x0304 已在该版本中删除，不再列出。 */
    GAME_STATE_CMD_ID                 = 0x0001, //比赛状态数据，1Hz
    GAME_RESULT_CMD_ID                = 0x0002, //比赛结果数据，比赛结束触发
    GAME_ROBOT_HP_CMD_ID              = 0x0003, //机器人血量数据，3Hz
    FIELD_EVENTS_CMD_ID               = 0x0101, //场地事件数据，1Hz
    REFEREE_WARNING_CMD_ID            = 0x0104, //裁判警告数据
    SENTRY_CMD_ID                     = 0x0120, //哨兵自主决策指令，是 0x0301 的子内容 ID，不是 cmd_id
    ROBOT_STATE_CMD_ID                = 0x0201, //机器人性能体系数据，10Hz
    POWER_HEAT_DATA_CMD_ID            = 0x0202, //实时底盘缓冲能量和射击热量数据，10Hz
    ROBOT_POS_CMD_ID                  = 0x0203, //机器人位置数据，1Hz
    BUFF_MUSK_CMD_ID                  = 0x0204, //机器人增益和底盘能量数据，3Hz
    ROBOT_HURT_CMD_ID                 = 0x0206, //伤害状态数据，伤害发生后触发
    SHOOT_DATA_CMD_ID                 = 0x0207, //实时射击数据，弹丸发射后触发
    PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208, //允许发弹量与剩余金币数，10Hz
    RFID_STATUS_ID                    = 0x0209, //机器人 RFID 模块状态，3Hz
    GROUND_ROBOT_POS_ID               = 0x020B, //地面机器人位置数据，1Hz（旧名 GOUND_ROBOT_POS_ID，拼写错误）
    SENTRY_DECISION_INFO_ID           = 0x020D, //哨兵自主决策信息同步，1Hz
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301, //机器人交互数据
    CUSTOM_CONTROLLER_CMD_ID          = 0x0302, //自定义控制器数据（图传链路）
    MAP_COMMAND_CMD_ID                = 0x0303, //地图命令数据
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  RSTEP_HEADER_SOF  = 0,
  RSTEP_LENGTH_LOW  = 1,
  RSTEP_LENGTH_HIGH = 2,
  RSTEP_FRAME_SEQ   = 3,
  RSTEP_HEADER_CRC8 = 4,
  RSTEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)
// clang-format on

#endif  //ROBOMASTER_PROTOCOL_H
