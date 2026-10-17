/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.h
  * @brief          : The header file of bsp_can.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DRV_GPIO_H
#define DRV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Externs ------------------------------------------------------------------*/
extern void bspGpioInit(void);

extern void bmi088AccelNsL(void);

extern void bmi088AccelNsH(void);

extern void bmi088GyroNsL(void);

extern void bmi088GyroNsH(void);

#ifdef __cplusplus
}
#endif

#endif //DRV_GPIO_H