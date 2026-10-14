/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef APP_REFEREE_H
#define APP_REFEREE_H
#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

typedef struct __packed
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint16_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} SendFrameHeader_t;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_task(void const * argument);

#ifdef __cplusplus
extern "C" {
#endif
extern void LF_UpdateReferee(uint16_t robot_id, uint16_t remaining_hp, uint8_t connected);
#ifdef __cplusplus
}
#endif

#endif
