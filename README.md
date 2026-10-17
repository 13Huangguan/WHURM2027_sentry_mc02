# WHURM2027_sentry_mc02

本文仅介绍 [RM2027-LFTemplate](RM2027-LFTemplate)，基于 STM32H723、STM32 HAL、
FreeRTOS 和 C++14 的 MC02 固件工程。当前应用包含 BMI088 姿态解算与云台电机控制，
其余设备和业务可沿现有分层扩展。

## 框架与目录

```text
RM2027-LFTemplate/
├─ Core/                         CubeMX 初始化、main 与 RTOS 任务创建
├─ Drivers/、Middlewares/         HAL、CMSIS-DSP、FreeRTOS、USB 等依赖
├─ RM2027-LFCore/
│  ├─ 1_Middleware/
│  │  ├─ Driver/                 CAN、UART、SPI、GPIO、Tick 驱动
│  │  ├─ Algorithm/              基础数学、PID、SMC、LPF、Kalman、四元数、Force
│  │  ├─ Matrix/                 固定尺寸矩阵与运行时矩阵视图
│  │  └─ Profiling/              可选 DWT 周期统计
│  ├─ 2_Device/                  BMI088、遥控器、电机与设备扩展
│  ├─ 3_LuojiaFox/               任务间共享的系统状态快照及 C/C++ 接口
│  ├─ 4_Application/             INS 姿态任务与电机控制应用
│  └─ core_template.cpp/.hpp     新模块模板
├─ tests/                        主机数学、协议、DMA 流程和计时器回归测试
├─ Makefile、Core.mk             工程构建与链接规则
└─ .clang-format、.clang-tidy     SJTU 格式化规则与代码检查配置
```

依赖方向为应用层 → 设备层 → 驱动/算法层 → HAL/CMSIS。
`main.c` 完成外设、遥控器和电机初始化后启动调度器；INS 任务再初始化 BMI088，
执行低通滤波和四元数 EKF，发布姿态状态。CAN 接收中断更新电机反馈，CAN 任务周期执行控制与发送。
任务间通过 `LuojiaFox::system_state` 发布、读取状态快照；该接口使用临界区，供任务上下文调用。

## 主要模块用法

- **矩阵**：包含 `mat_matrix.hpp`，用 `matrix::Matrix<行, 列>` 进行定长计算，
  用 `matrix::View` 借用连续行优先数组。1～4 列矩阵向量乘法采用展开内核，其余复用 CMSIS-DSP。
  矩阵逆运算需检查返回状态；视图版本会修改传入的临时矩阵。
- **Kalman/EKF**：先调用 `kalmanFilterInit()` 或 `quaternionEkfInit()`，再周期更新。
  初始化分配工作缓冲，更新阶段不分配堆内存；通过 `mat_status` 判断矩阵计算是否成功。
- **Force**：初始化执行器物理参数与分配矩阵后，每周期调用
  `setTarget()`、`setNow()`、`timCalculatePeriodElapsedCallback()`，
  再通过 `getTorque()` / `getOut()` 获取力矩或电机控制量。需由应用显式接入控制任务。
- **扩展业务**：设备协议放在 `2_Device`，控制流程放在 `4_Application`；
  HAL 回调和生成代码调用的入口保持 C 链接，复用已有驱动、数学和矩阵接口。

矩阵模块改编自 SJTU 交龙战队的矩阵/向量库，保留 [MIT 许可证](RM2027-LFTemplate/RM2027-LFCore/1_Middleware/Matrix/LICENSE)。
接口细节与性能说明见 [LFCore 文档](RM2027-LFTemplate/RM2027-LFCore/README.md)。

## 构建、验证与规范

准备 GNU Arm Embedded 工具链和 GNU Make；主机测试另需 Python 3 与支持 C++14 的 `g++`。
从仓库根目录执行：

```powershell
cd RM2027-LFTemplate
make -j4                         # 默认构建，产物位于 build/
make performance -j4             # 全工程 -O3，产物位于 build/performance/
python tests/run_host_tests.py    # 主机回归测试，无需连接硬件
```

固件产物名为 `LuojiaFox_H7_Template.elf/.hex/.bin`。LFCore 默认使用 `-O3 -fno-math-errno`，
保留 NaN 判断；调试可在独立目录构建 `make BUILD_DIR=build/debug LFCORE_OPT=-Og -j4`。
板上测时使用 `LFCORE_PROFILE=1` 的独立构建，查看 DWT 的平均/最大周期统计；
默认关闭统计，实际延时收益需结合中断负载和 Cache 在板上测量。

源文件采用 `xxx_xxx.cpp`，C++ 头文件采用 `.hpp`；函数使用小驼峰，变量/成员使用下划线，
类型使用大驼峰，宏和全局常量全大写。`SJTU.clang-format` 的内容已替换工程 `.clang-format`，
按标准文件名自动发现；命令行格式化使用 `clang-format --style=file -i <源文件>`。
`.clang-tidy` 负责命名与代码检查，完整规范见 [团队命名约定](RM2027-LFTemplate/style-team-main/advice.md)。
