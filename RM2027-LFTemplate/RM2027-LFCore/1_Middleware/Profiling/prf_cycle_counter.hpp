#ifndef PRF_CYCLE_COUNTER_HPP
#define PRF_CYCLE_COUNTER_HPP

#ifndef LFCORE_PROFILE
    #define LFCORE_PROFILE 0
#endif

#if LFCORE_PROFILE
    #include <cstdint>

    #include "stm32h7xx.h"

namespace profiling {

// Read after halting the core; the 64-bit total is not an atomic runtime snapshot.
struct CycleStats {
    volatile uint32_t last_cycles = 0;
    volatile uint32_t min_cycles = UINT32_MAX;
    volatile uint32_t max_cycles = 0;
    volatile uint32_t samples = 0;
    volatile uint64_t total_cycles = 0;

    void record(uint32_t cycles) {
        this->last_cycles = cycles;
        if (cycles < this->min_cycles) {
            this->min_cycles = cycles;
        }
        if (cycles > this->max_cycles) {
            this->max_cycles = cycles;
        }
        this->total_cycles += cycles;
        ++this->samples;
    }
};

extern CycleStats force_cycle_stats;
extern CycleStats kalman_cycle_stats;
extern CycleStats quaternion_cycle_stats;

inline void initCycleCounter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Cortex-M7 trace registers can start locked after reset.
    DWT->LAR = 0xC5ACCE55U;
    // Do not reset CYCCNT: the debugger or another profiler may already use it.
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
}

inline uint32_t readCycles() {
    __COMPILER_BARRIER();
    const uint32_t cycles = DWT->CYCCNT;
    __COMPILER_BARRIER();
    return cycles;
}

class CycleScope {
public:
    explicit CycleScope(CycleStats& stats): stats_(stats), start_(readCycles()) {}
    ~CycleScope() {
        this->stats_.record(readCycles() - this->start_);
    }
    CycleScope(const CycleScope&) = delete;
    CycleScope& operator=(const CycleScope&) = delete;

private:
    CycleStats& stats_;
    uint32_t start_;
};

} // namespace profiling
#endif // LFCORE_PROFILE

#endif // PRF_CYCLE_COUNTER_HPP
