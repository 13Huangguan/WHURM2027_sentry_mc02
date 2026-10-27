/**
 * @file app_chassis.h
 * @brief 四轮全向底盘应用对外接口。
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机、力控分配器和车体速度环。
 * @note 必须在 bspFdcanInit() 之后、FreeRTOS 调度器启动之前调用。
 *       任一电机配置或注册失败时会进入 Error_Handler()。
 */
void Chassis_Init(void);

/**
 * @brief 执行一次底盘控制周期。
 * @note 应在底盘控制任务中周期调用，不要在中断中调用。
 */
void Chassis_Control(void);

#ifdef __cplusplus
}
#endif

#endif
