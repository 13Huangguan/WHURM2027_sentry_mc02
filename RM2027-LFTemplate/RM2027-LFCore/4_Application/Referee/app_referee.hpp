/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       app_referee.cpp/hpp
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
#ifndef APP_REFEREE_HPP
#define APP_REFEREE_HPP

#include "main.h"
#include "referee.hpp"

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
  * @note           线程在 Core/Src/freertos.c（C 文件）里注册，必须保持 C 链接。
  */
extern "C" void referee_task(void const * argument);

#endif
