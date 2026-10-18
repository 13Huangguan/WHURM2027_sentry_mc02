/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.h
  * @brief          : The header file of bsptick.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DRV_TICK_H
#define DRV_TICK_H

#include "core_linkage.h"

#include "stdint.h"

LF_EXTERN_C void delayUs(uint32_t us);

LF_EXTERN_C void delayMs(uint32_t ms);

#endif