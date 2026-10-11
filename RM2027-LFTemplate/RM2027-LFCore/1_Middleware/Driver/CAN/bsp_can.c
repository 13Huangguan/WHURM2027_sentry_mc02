/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : FDCAN 收发底层驱动
  * @version        : v1.0
  * @date           : 2026/10/04
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

#include "fdcan.h"
#include "main.h"

/* Private function declarations ---------------------------------------------*/

static void BSP_FDCAN_FilterInit(FDCAN_HandleTypeDef *hcan);

/* Exported functions --------------------------------------------------------*/

void BSP_FDCAN_Init(void)
{
    BSP_FDCAN_FilterInit(&hfdcan1);
}

void BSP_FDCAN_InitTxFrame(FDCAN_TxFrame_TypeDef *frame, FDCAN_HandleTypeDef *hcan)
{
    if (frame == NULL)
    {
        return;
    }

    frame->hcan = hcan;

    frame->Header.IdType = FDCAN_STANDARD_ID;
    frame->Header.TxFrameType = FDCAN_DATA_FRAME;
    frame->Header.DataLength = FDCAN_DLC_BYTES_8;
    frame->Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    frame->Header.BitRateSwitch = FDCAN_BRS_OFF;
    frame->Header.FDFormat = FDCAN_CLASSIC_CAN;
    frame->Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    frame->Header.MessageMarker = 0;
    frame->Header.Identifier = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        frame->Data[i] = 0;
    }
}

void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *frame)
{
    if ((frame == NULL) || (frame->hcan == NULL))
    {
        return;
    }

    HAL_FDCAN_AddMessageToTxFifoQ(frame->hcan, &frame->Header, frame->Data);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief 标准帧全接收存进 Rx FIFO0，并打开新消息中断
  * @param hcan 目标 FDCAN 句柄
  */
static void BSP_FDCAN_FilterInit(FDCAN_HandleTypeDef *hcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//只想接收固定 CAN ID 时修改，FilterID1 = 期望匹配的 CAN 标识符，FilterID2 = 掩码，决定哪些位需要比较
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;   /* 掩码全 0 表示不比较任何位，即全部接收 */

    if (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 不匹配的帧与远程帧一律拒绝 */
    if (HAL_FDCAN_ConfigGlobalFilter(hcan,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* HAL callbacks -------------------------------------------------------------*/

/**
  * @brief Rx FIFO0 收到新消息后转交应用层
  * @param hfdcan   触发中断的 FDCAN 句柄
  * @param RxFifo0ITs 中断标志
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        {
            break;
        }

        BSP_FDCAN_RxHandler(hfdcan, rxHeader.Identifier, rxData);
    }
}

/**
  * @brief 应用层未实现 BSP_FDCAN_RxHandler 时的空实现
  */
__attribute__((weak)) void BSP_FDCAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t identifier, uint8_t *data)
{
    (void)hcan;
    (void)identifier;
    (void)data;
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
