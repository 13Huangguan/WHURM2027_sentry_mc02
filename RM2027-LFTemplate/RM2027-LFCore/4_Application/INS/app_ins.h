/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_INS_H
#define APP_INS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct
{
	float Pitch_Angle;
	float Yaw_Angle;
	float Yaw_TolAngle;
	float Roll_Angle;

  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

  float Angle[3];
	float Gyro[3];
	float Accel[3];

	float Last_Yaw_Angle;
	int16_t YawRoundCount;

}INS_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info;

/**
 * @brief the thread entry point registered by freertos.c.
 * @note  must be declared here, inside extern "C", so the definition in
 *        app_ins.cpp keeps C linkage and overrides the __weak stub in freertos.c.
 */
extern void INS_Task(void const * argument);

#ifdef __cplusplus
 }
#endif

#endif //APP_INS_H