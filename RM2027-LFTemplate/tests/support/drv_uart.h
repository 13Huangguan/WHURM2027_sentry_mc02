// Test-only HAL boundary for remote_control.cpp; production uses the workspace HAL.
#ifndef TEST_DRV_UART_H
#define TEST_DRV_UART_H

#include <cstdint>

struct DmaStream {
    uint32_t CR = 0;
    uint32_t NDTR = 0;
};
using DMA_Stream_TypeDef = DmaStream;
struct UartRegisters {
    uint32_t ISR = 0;
};
struct UART_HandleTypeDef {
    UartRegisters* Instance;
};
struct DMA_HandleTypeDef {
    void* Instance;
};
extern UartRegisters test_uart;
#define UART5 (&test_uart)
#define UART_FLAG_RXNE 1U
#define UART_FLAG_IDLE 2U
#define DMA_SxCR_EN 1U
#define DMA_SxCR_CT 2U
#define __HAL_UART_CLEAR_PEFLAG(handle) ((handle)->Instance->ISR = 0)
#define __HAL_UART_CLEAR_IDLEFLAG(handle) ((handle)->Instance->ISR = 0)
#define __HAL_DMA_DISABLE(handle) (static_cast<DmaStream*>((handle)->Instance)->CR &= ~DMA_SxCR_EN)
#define __HAL_DMA_ENABLE(handle) (static_cast<DmaStream*>((handle)->Instance)->CR |= DMA_SxCR_EN)

void rcInit(uint8_t* first, uint8_t* second, uint16_t size);
void rcRestart(uint16_t size);
uint32_t HAL_GetTick();
void SCB_InvalidateDCache_by_Addr(uint32_t* address, int32_t size);
void SCB_CleanInvalidateDCache_by_Addr(uint32_t* address, int32_t size);

#endif
