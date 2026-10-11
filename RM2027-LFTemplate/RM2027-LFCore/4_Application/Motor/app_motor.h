/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_motor.h
  * @brief          : 电机应用层：对象定义、接收分发、周期发送
  * @version        : v1.0
  * @date           : 2026/10/04
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported function declarations --------------------------------------------*/

/**
  * @brief 启动 CAN 并配置所有电机
  * @note  必须在 MX_FDCAN1/2/3_Init() 之后调用
  */
void Motor_Init(void);

/**
  * @brief 周期发送电机控制量
  * @note  由 CAN_Task 以 1kHz 调用
  */
void Motor_Control(void);

/**
  * @brief 设置云台 pitch 的目标角度
  * @param angle 目标角度，单位度
  * @note  上电软启动完成后调用才会生效
  */
void Motor_SetPitchAngle(float angle);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/* 以下接口仅 C++ 可见，需要读电机数据时包含本头文件 */
#include "Motor.hpp"

extern DJI_Motor_Info_Class g_pitchMotor;     /*!< 云台 pitch，GM6020 */
#endif

#endif /* APP_MOTOR_H */

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
