#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "drv_uart.h"
#include "remote_control.h"
#include "stm32h7xx_it.h"

UartRegisters test_uart;
DmaStream test_dma;
UART_HandleTypeDef huart5 { &test_uart };
DMA_HandleTypeDef hdma_uart5_rx { &test_dma };

namespace {
uint8_t* buffers[2];
uint32_t tick = 100;
int invalidations = 0;
int clean_invalidations = 0;
void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        std::exit(1);
    }
}
} // namespace

uint32_t HAL_GetTick() {
    return tick;
}
void rcInit(uint8_t* first, uint8_t* second, uint16_t size) {
    buffers[0] = first;
    buffers[1] = second;
    check(size == SBUS_RX_BUF_NUM, "cache padding must not change DMA transfer length");
    check(reinterpret_cast<uintptr_t>(first) % 32 == 0, "first DMA row aligned");
    check(reinterpret_cast<uintptr_t>(second) % 32 == 0, "second DMA row aligned");
    check(second - first == 64, "DMA cache rows isolated");
    test_dma.NDTR = size;
    test_dma.CR = DMA_SxCR_EN;
}
void rcRestart(uint16_t size) {
    test_dma.NDTR = size;
}
void SCB_InvalidateDCache_by_Addr(uint32_t* address, int32_t size) {
    ++invalidations;
    check(
        size == 64 && reinterpret_cast<uintptr_t>(address) % 32 == 0,
        "invalidate whole cache rows"
    );
}
void SCB_CleanInvalidateDCache_by_Addr(uint32_t* address, int32_t size) {
    ++clean_invalidations;
    check(
        size == 128 && reinterpret_cast<uintptr_t>(address) % 32 == 0,
        "prepare DMA cache before receive"
    );
}

int main() {
    remoteControlInit();
    check(clean_invalidations == 1, "DMA initialization cache maintenance");
    check(getRcOffline(), "initially offline");
    for (int frame_index = 0; frame_index < 8; ++frame_index) {
        const int completed = frame_index % 2;
        std::memset(buffers[completed], 0, 36);
        // A centered DT7 frame, with two valid switches and zeroed roller.
        for (int channel = 0; channel < 4; ++channel) {
            const int bit = channel * 11 + 10;
            buffers[completed][bit / 8] |= 1 << (bit % 8);
        }
        buffers[completed][5] |= (RC_SW_DOWN | (RC_SW_MID << 2)) << 4;
        buffers[completed][17] = 4;
        test_dma.NDTR = SBUS_RX_BUF_NUM - RC_FRAME_LENGTH;
        tick += 1;
        test_uart.ISR = UART_FLAG_IDLE;
        UART5_IRQHandler();
        check(test_dma.NDTR == SBUS_RX_BUF_NUM, "DMA length restored for next frame");
        check(((test_dma.CR & DMA_SxCR_CT) != 0) == (completed == 0), "DMA buffer toggled");
        check(getDt7RcSw(0) == RC_SW_DOWN, "both DMA buffers decoded");
    }
    check(invalidations == 8, "each completed row invalidated once");
    check(!getRcOffline(), "valid consecutive frames restore connection");
    tick += 101;
    check(getRcOffline(), "timeout goes offline");
    check(getDt7RcCh(255) == 0 && getDt7RcSw(255) == 0 && !getDt7Keyboard(255), "API bounds");
    std::puts("PASS: remote DMA length/cache isolation/alternation/timeout checks");
}
