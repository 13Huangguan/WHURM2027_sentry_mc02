
#ifndef BSP_UART_H
#define BSP_UART_H

#include "core_linkage.h"

#include "stm32h7xx_hal.h"
#include "usart.h"

typedef UART_HandleTypeDef UartHandle;

typedef enum UartSendStateTag {
    UART_SEND_FAIL = 0,
    UART_SEND_OK,
} UartSendState;

LF_EXTERN_C UartSendState
uartSendTxMessage(UART_HandleTypeDef* huart, uint8_t* p_data, uint16_t size, uint32_t timeout);

LF_EXTERN_C void rcInit(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_num);
LF_EXTERN_C void rcUnable(void);
LF_EXTERN_C void rcRestart(uint16_t dma_buf_num);

LF_EXTERN_C void usart1_init(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_num);

#endif // BSP_UART_H
