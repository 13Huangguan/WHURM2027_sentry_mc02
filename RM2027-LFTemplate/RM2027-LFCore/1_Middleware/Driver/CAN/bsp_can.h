/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : FDCAN 收发底层驱动
  * @version        : v1.0
  * @date           : 2026/10/04
  * @attention      : 标准帧收发、长度校验、观察者与诊断，不含设备标识符。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
    #define BSP_CAN_H

    #include "core_linkage.h"

/* Includes ------------------------------------------------------------------*/

    #include <stdbool.h>

    #include "stm32h7xx_hal.h"

/* Exported function declarations --------------------------------------------*/

/**
  * @brief 配置滤波器、启动三路 FDCAN 并使能接收中断
  * @note  必须在 MX_FDCAN1/2/3_Init() 之后调用；
  *        只使用部分总线时可直接调用 bspFdcanInitBus()。
  */
LF_EXTERN_C void bspFdcanInit(void);

/**
  * @brief 初始化单路 FDCAN
  * @param hcan FDCAN 句柄
  */
LF_EXTERN_C void bspFdcanInitBus(FDCAN_HandleTypeDef* hcan);

/**
  * @brief 按编号取 FDCAN 句柄
  * @param index 1~3
  * @return 句柄；编号非法时返回 NULL
  */
LF_EXTERN_C FDCAN_HandleTypeDef* bspFdcanGetHandle(uint8_t index);

/**
  * @brief 直接发送一帧标准数据帧
  * @param hcan       FDCAN 句柄
  * @param identifier 标准标识符
  * @param data       数据指针
  * @param length     数据长度，0~8
  * @return true 表示已成功写入 Tx FIFO
  */
LF_EXTERN_C bool bspFdcanSend(
    FDCAN_HandleTypeDef* hcan,
    uint32_t identifier,
    const uint8_t* data,
    uint8_t length
);

/**
  * @brief 接收回调，由应用层实现
  * @param hcan       收到数据的 CAN 句柄
  * @param identifier 标准标识符
  * @param data       数据仅在回调期间有效
  * @param length     实际数据长度
  * @note  应用层不实现也能编译通过，此时收到的帧被丢弃
  */
LF_EXTERN_C void bspFdcanRxHandler(FDCAN_HandleTypeDef* hcan, uint32_t identifier,
                                  const uint8_t* data, uint8_t length);

/** @brief 非电机设备可注册最多 8 个观察者；初始化时注册，在 CAN ISR 内同步调用。
 * data/header 仅在回调期间有效，禁止阻塞、发送 CAN 或调用 RTOS 阻塞接口。
 * 观察者接收标准数据帧（含 FD），电机桥仅接收经典 CAN 帧。
 */
typedef void (*BspFdcanObserver)(FDCAN_HandleTypeDef*, const FDCAN_RxHeaderTypeDef*,
                                const uint8_t*, uint8_t);
LF_EXTERN_C bool bspFdcanAddObserver(BspFdcanObserver observer);

typedef struct {
    uint32_t tx_queued;
    uint32_t tx_failed;
    uint32_t rx_frames;
    uint32_t rx_lost;
    uint32_t errors;
    uint32_t bus_off_count;
    uint32_t recoveries;
    uint32_t last_hal_error;
    bool bus_off;
    bool recovering;
} BspFdcanStats;

LF_EXTERN_C bool bspFdcanGetStats(FDCAN_HandleTypeDef* hcan, BspFdcanStats* stats);
/** @brief 任务上下文显式恢复：重新初始化并清空旧 TX 请求。
 * 调用前停止本总线所有发送者；恢复不会重新使能电机，调用方应复位反馈并重新确认在线。
 * 保留周期控制帧不自动重发的策略；使能等事务命令由应用确认反馈后有限重试。
 */
LF_EXTERN_C bool bspFdcanRecover(FDCAN_HandleTypeDef* hcan);

#endif /* BSP_CAN_H */

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
