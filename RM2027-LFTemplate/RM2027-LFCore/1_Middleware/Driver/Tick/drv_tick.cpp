/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.c
  * @brief          : HAL delay functions 
  * @author         : Yan Yuanbin
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

/* Include------------------------------------------------------------------*/
#include "drv_tick.h"

#include "stm32h7xx.h"

/**
  * @brief  report the microsecond haltick
  * @param  none
  * @note   the cubemx recommended to switch the system time base to a timer other than SysTick, 
  *         so there will be two sets of time bases in the system, 1.SysTick for RTOS 2.HalTick for HAL
  *         SysTick uses the SysTick of the cortex-m4 kernel (SysTick->VAL will be updated after starting the task scheduler)
  *         HalTick uses TIM2 in this project (TIM2->CNT can provide microsecond delay)
  *         # delay_us and delay_ms will not cause task scheduling (blocking type)
  * @retval haltick
  */
static uint32_t haLTick(void) {
    uint32_t tick_us = 0;

    uint32_t ms = 0, us = 0;

    ms = HAL_GetTick();
    // use the TIM2 as the HAL TimeBase
    // Freq:1MHz => 1Tick = 1us
    // Period:1ms

    us = TIM2->CNT;

    tick_us = ms * 1000 + us;

    return tick_us;
}

/**
  * @brief  microsecond delay
  * @param  us: delay microsecond 
  * @retval none
  */
void delayUs(uint32_t us) {
    uint32_t now = haLTick();

    while ((haLTick() - now) < us) {}
}

/**
  * @brief  millisecond delay
  * @param  ms: delay millisecond
  * @retval none
  */
void delayMs(uint32_t ms) {
    uint32_t now = HAL_GetTick();

    while ((HAL_GetTick() - now) < ms) {}
}