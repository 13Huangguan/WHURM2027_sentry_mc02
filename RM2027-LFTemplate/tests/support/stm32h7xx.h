// Fake cycle-counter registers for the host profiler regression only.
#ifndef TEST_STM32H7XX_H
#define TEST_STM32H7XX_H

#include <cstdint>

struct TestDwt {
    volatile uint32_t CTRL = 0;
    volatile uint32_t CYCCNT = 0;
    volatile uint32_t LAR = 0;
};
struct TestCoreDebug {
    volatile uint32_t DEMCR = 0;
};
extern TestDwt test_dwt;
extern TestCoreDebug test_core_debug;
#define DWT (&test_dwt)
#define CoreDebug (&test_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __COMPILER_BARRIER() __asm__ volatile("" ::: "memory")
inline void __DSB() {}
inline void __ISB() {}

#endif
