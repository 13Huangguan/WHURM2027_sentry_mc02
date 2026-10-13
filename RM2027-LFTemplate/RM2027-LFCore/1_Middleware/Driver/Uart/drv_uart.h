
#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32h7xx_hal.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef UART_HandleTypeDef huart_t;

typedef enum __UartSendState {
    UART_SEND_FAIL = 0,
    UART_SEND_OK,
} UartSendState_e;

extern UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout);

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

#ifdef __cplusplus
}
#endif

#endif  // BSP_UART_H
