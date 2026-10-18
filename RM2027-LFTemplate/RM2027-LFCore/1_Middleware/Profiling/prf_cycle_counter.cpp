#include "prf_cycle_counter.hpp"

#if LFCORE_PROFILE
namespace profiling {
CycleStats force_cycle_stats;
CycleStats kalman_cycle_stats;
CycleStats quaternion_cycle_stats;
} // namespace profiling
#endif
