/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : FDCAN 收发底层驱动
  * @version        : v1.0
  * @date           : 2026/10/04
  * @attention      : 提供 Motor.hpp / Motor.cpp 需要的 FDCAN_TxFrame_TypeDef
  *                   与发送函数，不含任何写死的标识符。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief FDCAN 发送帧
  */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;      /*!< 该帧由哪路 CAN 发出 */
    FDCAN_TxHeaderTypeDef Header;   /*!< 发送头，标识符填在 Header.Identifier */
    uint8_t Data[8];                /*!< 8 字节数据 */
} FDCAN_TxFrame_TypeDef;

/**
  * @brief FDCAN 接收帧
  */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;      /*!< 来自哪路 CAN */
    FDCAN_RxHeaderTypeDef Header;   /*!< 接收头，标识符在 Header.Identifier */
    uint8_t Data[8];                /*!< 8 字节数据 */
} FDCAN_RxFrame_TypeDef;

/* Exported function declarations --------------------------------------------*/

/**
  * @brief 配置滤波器、启动三路 FDCAN 并使能接收中断
  * @note  必须在 MX_FDCAN1/2/3_Init() 之后调用
  */
void BSP_FDCAN_Init(void);

/**
  * @brief 把发送帧初始化为默认状态（句柄 + 标准帧 + 8 字节）
  * @param frame 待初始化的发送帧
  * @param hcan  该帧使用的 FDCAN 句柄
  */
void BSP_FDCAN_InitTxFrame(FDCAN_TxFrame_TypeDef *frame, FDCAN_HandleTypeDef *hcan);

/**
  * @brief 把发送帧丢进 Tx FIFO
  * @param frame 已填好 Header.Identifier 和 Data 的发送帧
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *frame);

/**
  * @brief 接收回调，由应用层实现
  * @param hcan       收到数据的 CAN 句柄
  * @param identifier 标准标识符
  * @param data       8 字节数据
  * @note  应用层不实现也能编译通过，此时收到的帧被丢弃
  */
void BSP_FDCAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t identifier, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/