#include <cstdio>
#include <cstdlib>

#include "prf_cycle_counter.hpp"

TestDwt test_dwt;
TestCoreDebug test_core_debug;

void check(bool condition, const char* label) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        std::exit(1);
    }
}

int main() {
    test_dwt.CYCCNT = 100;
    profiling::initCycleCounter();
    check(test_dwt.CYCCNT == 100, "initialization preserves counter");
    check(test_dwt.LAR == 0xC5ACCE55U, "Cortex-M7 trace lock unlocked");
    check((test_dwt.CTRL & 1) != 0 && (test_core_debug.DEMCR & (1U << 24)) != 0, "counter enabled");
    auto& stats = profiling::force_cycle_stats;
    {
        profiling::CycleScope scope(stats);
        test_dwt.CYCCNT += 25;
    }
    check(
        stats.last_cycles == 25 && stats.min_cycles == 25 && stats.max_cycles == 25,
        "first sample"
    );
    test_dwt.CYCCNT = 0xfffffff0U;
    {
        profiling::CycleScope scope(stats);
        test_dwt.CYCCNT = 0x10U;
    }
    check(
        stats.last_cycles == 32 && stats.min_cycles == 25 && stats.max_cycles == 32,
        "counter rollover"
    );
    check(stats.samples == 2 && stats.total_cycles == 57, "sample accumulation");
    std::puts("PASS: cycle profiler enable/preservation/rollover/statistics checks");
}
