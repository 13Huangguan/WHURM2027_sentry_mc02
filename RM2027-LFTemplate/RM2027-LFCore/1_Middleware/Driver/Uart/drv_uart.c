/**
 * @brief uart驱动层
 * @author Bayekq
 */
#include "drv_uart.h"
#include "arm_math.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/**
 * @brief 使用uart发送数据
 * @param huart 
 * @param pData 
 * @param Size 
 * @param Timeout 
 * @return 
 */
UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t cnt = 5;  //最大重发次数
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    while (cnt-- && status != HAL_OK) {
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    }

    if (status == HAL_OK) {
        return UART_SEND_OK;
    }
    return UART_SEND_FAIL;
}
/**
 * @brief RC串口初始化
 * @param rx1_buf 接收缓冲区1
 * @param rx2_buf 接收缓冲区2
 * @param dma_buf_num DMA接收长度
 * @return 
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    while(((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }

    ((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->PAR = (uint32_t) & (UART5->RDR);
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    ((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);

}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart5);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef* )hdma_uart5_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);

}


