"""Compile production algorithms against a test-only scalar CMSIS adapter, then run regressions."""
from pathlib import Path
import os
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "RM2027-LFCore"
OUTPUT = ROOT / "build" / "host_tests"
OUTPUT.mkdir(parents=True, exist_ok=True)
COMPILER = os.environ.get("HOST_CXX", "g++")
INCLUDES = [ROOT / "tests/support", CORE / "1_Middleware/Algorithm/Matrix",
            CORE / "1_Middleware/Algorithm/Basic", CORE / "1_Middleware/Algorithm/Force",
            CORE / "1_Middleware/Algorithm/Kalman_Filter", CORE / "1_Middleware/Algorithm/LPF",
            CORE / "1_Middleware/Algorithm/Quatertion", CORE / "4_Application/INS",
            CORE / "2_Device/RC", CORE / "3_LuojiaFox", CORE / "1_Middleware/Profiling"]
FLAGS = ["-std=c++14", "-O3", "-fno-math-errno", "-Wall", "-Wextra", "-Werror", "-fno-exceptions", "-fno-rtti"]
FLAGS += ["-I" + str(path) for path in INCLUDES]


def run_test(name, sources, flags=()):
    binary = OUTPUT / (name + (".exe" if os.name == "nt" else ""))
    subprocess.run([COMPILER, *FLAGS, *flags, *map(str, sources), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


run_test("core_math_test", [ROOT / "tests/core_math_test.cpp",
    CORE / "1_Middleware/Algorithm/Matrix/mat_matrix.cpp",
    CORE / "1_Middleware/Algorithm/Basic/alg_basic.cpp",
    CORE / "1_Middleware/Algorithm/Force/alg_force.cpp",
    CORE / "1_Middleware/Algorithm/Kalman_Filter/alg_kalman_filter.cpp",
    CORE / "1_Middleware/Algorithm/Quatertion/alg_quaternion.cpp",
    CORE / "1_Middleware/Algorithm/LPF/alg_lpf.cpp"])
for rc_type in range(4):
    run_test(f"rc_protocol_{rc_type}_test", [ROOT / "tests/rc_protocol_test.cpp",
        CORE / "2_Device/RC/rc_protocol.cpp"], [f"-DRC_TYPE={rc_type}"])

run_test("rc_dma_test", [ROOT / "tests/rc_dma_test.cpp",
    CORE / "2_Device/RC/remote_control.cpp", CORE / "2_Device/RC/rc_protocol.cpp"],
    ["-DRC_TYPE=0"])

run_test("profile_counter_test", [ROOT / "tests/profile_counter_test.cpp",
    CORE / "1_Middleware/Profiling/prf_cycle_counter.cpp"], ["-DLFCORE_PROFILE=1"])
