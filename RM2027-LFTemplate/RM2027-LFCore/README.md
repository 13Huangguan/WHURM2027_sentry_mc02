## 编码规范

以 `../style-team-main/guide.md` 和 `../style-team-main/advice.md` 为准：

- C++ 源文件采用 `xxx_xxx.cpp`，C++ 接口采用 `.hpp`。
- 类型使用 `CamelCase`，函数及成员函数使用小驼峰 `lowerCamelCase`；变量、成员和命名空间使用 `lower_case`。
- 宏、全局常量及枚举成员使用 `UPPER_CASE`。
- 使用 4 空格缩进、100 列行宽；工程 `.clang-format` 已用 `SJTU.clang-format` 的内容替换，
  按标准文件名自动发现，无需 VS Code 自定义格式配置路径。
  命令行在 `RM2027-LFTemplate` 下执行 `clang-format --style=file -i <源文件>`。
- HAL、CMSIS 和 FreeRTOS 定义的类型、成员和回调名称遵循外部接口要求。
- 供生成的 C 代码调用的接口保留 C 兼容 `.h`；C++ 任务入口和 HAL 回调显式使用 C 链接。

## C++ 迁移

LFCore 内的 8 个 C 源文件已经迁移到 C++，生成的 HAL、USB 和 RTOS C 源文件仍由顶层 Makefile 构建。

| 原文件 | 当前文件 |
| --- | --- |
| `alg_kalman_filter.c/.h` | `alg_kalman_filter.cpp/.hpp` |
| `Quaternion.c/.h` | `alg_quaternion.cpp/.hpp` |
| `Bmi088.c/.h` | `dvc_bmi088.cpp/.hpp` |
| `bsp_can.c` | `bsp_can.cpp` |
| `drv_spi.c` | `drv_spi.cpp` |
| `drv_tick.c` | `drv_tick.cpp` |
| `drv_uart.c` | `drv_uart.cpp` |
| `remote_control.c` | `remote_control.cpp`，协议解析共用 `rc_protocol.cpp` |

现有源文件也统一了文件名：`Motor` → `dvc_motor`，`LF` → `lf_core`，
`smc` → `alg_smc`，`template` → `core_template`。所有工程内引用已同步更新。
基础数学、Force 和 LPF 头文件分别改为 `alg_basic.hpp`、`alg_force.hpp`、`alg_lpf.hpp`。

迁移前的构建目录中可能留有指向 `.c` 的依赖文件，首次在其他工作副本中构建时请先清理旧产物，
或使用独立构建目录：在 `RM2027-LFTemplate` 下执行 `make BUILD_DIR=build/cpp_migration -j4`。

## CAN 电机通用模板

完整接口约定、方向与减速比、快照、故障状态及 CAN 恢复流程见
[电机与 CAN 使用说明](2_Device/Motor/MOTOR_USAGE.md)。

模板不包含 `app_motor` 应用层和独立电机任务。底盘、云台、发射等应用在自己的模块中
创建/注册电机，并在自己的控制周期里调用发送函数。

| 文件 | 职责 |
| --- | --- |
| `dvc_motor.hpp/.cpp` | `MotorBase` 公共接口、静态注册、CAN 接收分发 |
| `dvc_dji_motor.hpp/.cpp` | DJI GM6020 / M3508 / M2006 的解析与统一发送 |
| `dvc_dm_motor.hpp/.cpp` | 达妙 MIT / 位置速度 / 速度控制 |
| `dvc_lk_motor.hpp/.cpp` | 瓴控 MF / MG 的使能、电流、速度、位置控制 |

### 注册电机

```cpp
#include "dvc_dji_motor.hpp"

static DjiMotor motor;

// 初始化阶段
motor.init(MotorType::DjiM3508, 1U, &hfdcan1, 1, 0.0f);
MotorBase::registerMotor(motor);   // 注册后才能接收反馈

// 控制周期
motor.setCommand(5000);
DjiMotor::sendGroup(&hfdcan1, 0x200);
```

`MotorBase::onCanFrame()` 已由 `dvc_motor.cpp` 里的 `bspFdcanRxHandler()` 接好，
应用层不需要再写接收分发。在线状态用 `motor.isOnline()` / `motor.lastRxTick()`。

### DJI 标识符

由类型和拨码 ID 推导，不需要手写：

| 类型 | 接收标识符 | 发送标识符（ID 1~4 / 5~8） |
| --- | --- | --- |
| GM6020 | `0x205 + ID - 1` | `0x1FF / 0x2FF` |
| M3508、M2006 | `0x201 + ID - 1` | `0x200 / 0x1FF` |

`DjiMotor::sendGroup(bus, identifier)` 一次只发一个控制帧：
从注册表汇总同一路总线上 `txIdentifier() == identifier` 的全部 DJI 电机，按 `txSlot()` 放进 0~3 槽。
不同任务可以各自负责自己的 `0x200` / `0x1FF` / `0x2FF`，互不影响。

如果已经自己排好 4 台电机的控制量，可以直接调用静态的
`DjiMotor::sendFrame(bus, identifier, current[4])` 发一帧，不经过电机对象。

### 瓴控电机（LK / MF / MG）

`LkMotor` 支持使能/失能/停止、电流、速度、位置控制，并解析驱动器自动上报的
`0x9C` 状态帧。

- 命令和反馈标识符：`0x140 + ID`（ID 1~32）。
- 编码器位数按型号传 14/15/16，默认 16。
- 电流比例：MF 为 `33/4096 A/LSB`，MG 为 `66/4096 A/LSB`，
  通过 `init()` 的 `mg_current` 选择。
- 速度/位置内部按协议换算为 `0.01 dps/LSB` 和 `0.01 deg/LSB`。
- 当前板级 FDCAN 为 **1 Mbps**（PLL2Q = 100 MHz，预分频 5 / 20 tq），
  LK 驱动器需要用 LK 上位机设置为相同的 1 Mbps。

```cpp
#include "dvc_lk_motor.hpp"

static LkMotor dial_motor;

// 初始化：ID1，FDCAN1，16 位编码器，MF 系列
dial_motor.init(1U, &hfdcan1, 16U, false);
MotorBase::registerMotor(dial_motor);

// 使能并速度控制
dial_motor.sendEnable();
dial_motor.sendSpeed(20.0f);          // 20 rad/s，输出轴

// 或位置控制：目标 1.57 rad，最大 3.14 rad/s（约 180 deg/s）
dial_motor.sendPosition(1.57f, 3.141592654f);

// 反馈
const auto fdb = dial_motor.data();
const bool online = dial_motor.isOnline();
// fdb.angle_rad / fdb.velocity_rad_s / fdb.current_a / fdb.temperature
```

`sendSpeed()` 的第二个参数是最大转矩电流幅值（A），MF 上限 16.5 A、MG 上限 33 A。
0 原样发送编码 0，其具体行为需核实驱动固件。`sendPosition()` 接收多圈 rad 目标及 rad/s 速度上限。
驱动器不主动上报时可以周期调用 `dial_motor.sendReadStatus()`。

测试模块：`4_Application/Motor/app_lk_test.cpp` 默认输出 0.5 Hz、幅值 3 rad/s 的
周期正弦速度，持续 10 秒。模板默认不在 `main.c` 中调用 `lkTestInit()`，需要测试时手动启用；目标速度和反馈可在
`g_lk_test_feedback` 中观察。测试参数在该文件顶部修改。

其他 CAN 设备可直接调用 `bspFdcanSend()`。新增厂商电机时继承 `MotorBase`，
实现 `update()`/`reset()` 后调用 `MotorBase::registerMotor()`。

## 超级电容

新增设备驱动 `2_Device/Supercap` 与周期应用 `4_Application/Supercap`，按提供的 `superpower_task` 和文档移植 `0x61` 协议。线程在 `Core/Src/freertos.c` 中定义和创建。
默认 CAN1，反馈 ID 待填写宏，未配置时不启动任务。
接入说明、数据过滤及底盘预算接口见 [超级电容说明](4_Application/Supercap/README.md)。

## 2006 SMC 在线调参

调参模块、上位机工具和实测记录见独立文档 [SMC_TUNING.md](4_Application/Motor/SMC_TUNING.md)。

## 矩阵与滤波

`1_Middleware/Algorithm/Matrix/mat_matrix.hpp/.cpp` 改编自
`Matrix_and_Robotics_on_STM32-main/src/matrix.h/.cpp` 的矩阵、向量部分，保留原作者
Team JiaoLong-SJTU / Spoon Guan 的署名和 MIT 许可证（同目录 `LICENSE`）。
运算内核复用工程现有 CMSIS-DSP；限幅、符号和单位转换复用 `alg_basic.hpp`，
未引入另一套重复的数学工具函数。

- `matrix::Matrix<ROWS, COLS>`：栈上固定大小矩阵，支持复制、加减、乘法、缩放、转置、
  子矩阵、迹和范数。复制会重新绑定自己的数据；`matrix::inverse(input, output)` 返回状态码，
  保留调用者输入，奇异矩阵返回 `ARM_MATH_SINGULAR`。
- `matrix::View`：借用连续行优先数组的运行时矩阵视图，不分配内存、不复制数据。
  乘法和转置的输出不能与输入重叠；逆运算的输入是会被 CMSIS 修改的临时缓冲。
- Kalman/EKF 共用创新协方差和增益计算。初始化集中分配一块缓冲，重新初始化会释放旧缓冲；
  更新阶段不分配内存。临时缓冲按状态和测量维度的最大值分配，支持测量维度大于状态维度。
  矩阵运算失败时保留最近发布的输出，并通过 `mat_status` 报告失败；调用方应检查状态。
- 四元数 EKF 修复卡方检验的残差转置，使用独立的增益缩放值，避免临时矩阵覆盖它；
  零加速度、非有限输入和无效周期不推进状态。
- Force 的分配矩阵使用紧凑存储和公共矩阵向量乘法，直接写入力矩缓冲。
  初始化时缓存周期倒数、力矩限幅和电机换算系数；重力补偿为零时跳过余弦计算。
  非线性补偿仍逐执行器处理，运行时没有矩阵打包、堆分配或重复结果复制。

## 性能构建与延时测量

LFCore 默认使用 `-O3 -fno-math-errno`；生成的 HAL/RTOS 源码仍遵循顶层 `OPT`。
`-fno-math-errno` 允许编译器直接使用硬件平方根，本工程的控制代码不读取数学函数的 errno。
保留 IEEE 非有限值判断，不启用 `-Ofast` 或 `-ffast-math`。
需要逐语句调试时，在独立目录构建：

```sh
make BUILD_DIR=build/debug LFCORE_OPT=-Og -j4
```

完整性能固件使用独立目录，HAL/RTOS 与 LFCore 均使用 `-O3`：

```sh
make performance -j4
```

结果在 `build/performance`。切换编译选项时使用独立构建目录，避免复用不同选项的旧对象文件。
`-O3` 可能增大代码体积，也会改变调试时可观察的局部变量；它不意味着已经测得更低延时。

当前消除的固定开销包括：

- 公共矩阵向量乘法对 1～4 列展开累加，避免小矩阵进入通用矩阵乘法内核。
  其他列数继续复用 CMSIS-DSP，求和顺序保持不变。
- Force 不再每周期构造三份矩阵描述符；初始化时缓存惯量/周期和电机换算系数。
- 矩阵复制和完全覆盖的结果不再先清零；普通默认构造及非法子块仍返回零矩阵。
- EKF 复用加速度平方范数，并直接比较平方阈值，去掉陀螺仪求根、倒数和重复除法。
  原来只用于阈值判断的 `QuaternionInfo::gyro_inv_norm` 字段已移除。
  零陀螺仪读数正确识别为静止；不再依赖浮点除零的结果。
- EKF 只求实际使用的两个方向角，省去一个 `acosf`；控制代码的取模/幂/绝对值使用单精度函数。

需要上板测量时构建：

```sh
make BUILD_DIR=build/profile OPT=-O3 LFCORE_PROFILE=1 -j4
```

`1_Middleware/Profiling/prf_cycle_counter.hpp` 使用 Cortex-M7 DWT 的 `CYCCNT`，
不重置已启用的计数器。初始化 Force 或 Kalman 时启用计数，在函数退出时记录周期数。
暂停目标后可查看：

- `profiling::force_cycle_stats`
- `profiling::kalman_cycle_stats`
- `profiling::quaternion_cycle_stats`

每组记录 `last_cycles`、`min_cycles`、`max_cycles`、`samples` 和 `total_cycles`。
平均周期为 `total_cycles / samples`；微秒数为 `cycles * 1000000 / SystemCoreClock`。
记录汇总同一函数的所有实例与调用，包括失能和无效输入的快速返回；应在稳定运行阶段取样。
统计包含中断抢占，采样时不关闭中断；EKF 的耗时包含嵌套 Kalman 调用及其统计开销。
32 位 DWT 差值支持一次计数回绕，但单次测量不能超过一整个计数周期。
统计计数器应在一次有限时长的测量内使用，`samples` 回绕前重新开始测量。
64 位总数需暂停目标后读取以避免读取撕裂。

默认 `LFCORE_PROFILE=0` 时，统计字段和读计数器的代码都不进入固件。
未被应用调用的解算函数及对应统计记录会被链接器回收；例如应用未接入 Force 时没有 Force 统计项。
板上真实耗时仍取决于 Cache、存储位置、输入维度与中断负载，不能用主机测试或代码体积推算加速倍数。

Force 的典型调用：

```cpp
#include "alg_force.hpp"

ForceSolver solver;
ForceActuatorParam parameters[4] {};
// 填写 parameters 中的力矩常数、减速比、方向和力矩限幅等物理参数。
solver.init(parameters, 4, 3, 0.001f);
solver.setDistribution(distribution); // 连续的 4 x 3 行优先数组。
solver.setTarget(target);            // 长度 3。
solver.setNow(position, velocity);   // 各长 4；不更新的反馈可以传 nullptr。
solver.setEnable(true);
solver.timCalculatePeriodElapsedCallback();
float current = solver.getOut(0);
```

## 验证

在 `RM2027-LFTemplate` 下运行 `python tests/run_host_tests.py`。
测试直接编译生产算法，使用 `tests/support/arm_math.h` 的主机标量替身，不要求连接硬件。
覆盖矩阵所有权及求逆、Force 各尺寸与原标量公式的对照、全向底盘力重建、Kalman 不同维度、
奇异矩阵、四元数静止及旋转、卡方统计量、LPF 重置、四种遥控器协议和 DMA 缓冲切换。

主机测试验证数值和控制流程，不测量 Cortex-M7 的 DSP 性能，也不能替代实际传感器、DMA、
中断和电机联调。Force 的实际耗时应在目标板上用 DWT 周期计数器测量。
