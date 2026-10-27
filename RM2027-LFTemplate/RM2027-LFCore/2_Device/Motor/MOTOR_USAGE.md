# 电机与 CAN 模板使用约定

模板默认不调用 `lkTestInit()` 或 `smcTuneInit()`，当前调参入口为 `Tuning/bench_motors.cpp`。
历史独立 SMC 示例依赖已移除的 `ReceiveDataSmcDebug_s` 等旧 USB 接口，单独启用
`LEGACY_MOTOR_TESTS` 编译它之前需先恢复相应接口；本次同步单位未恢复旧协议。
LK 测试显式启用后，等待有效反馈才使能，离线即停止，默认以 3 rad/s、0.5 Hz 运行 10 秒。
SMC 测试显式启用后，仍需 USB 指令使能；USB 或电机离线时输出归零。

## 初始化与生命周期

每个机器人在应用初始化中集中配置自己的 CAN 口、型号、ID、方向、减速比、反馈超时。
电机对象使用 `static` 或全局存储，先检查 `init()`，再检查 `registerMotor()` 的返回值。
注册后不能再次 `init()`，也不能改变方向或减速比；禁止复制电机对象。
配置失败时不要创建对应控制任务。`reset()` 清空反馈、计数和离线时间；DJI 同时清空指令。

注册会检查同总线反馈 ID、发送 ID/槽位以及设备间发送与反馈 ID 的冲突。
DM/LK 独占自己的发送帧；DJI 允许同一帧内不同槽位。不同总线可使用相同 ID。
DM 反馈 ID 按电机上位机实际配置填写，支持独立的 ID 0；不支持多台电机共用一个反馈 ID。

## 方向、单位与减速比

`direction = -1` 同时翻转机构坐标反馈与目标指令。`reduction_ratio` 是协议报告轴到
机构输出轴的减速比。只在配置阶段设置，合法范围为 `1e-6 ~ 1e6`。

| 接口 | 单位与含义 |
| --- | --- |
| DJI `setCommand()` | 幅值仍为原始电流/电压控制单位，但符号按机构方向；发送时应用方向 |
| DJI `sendFrame()` | 底层原始槽位接口，不应用方向、不限幅；不要与同 ID 的 `sendGroup()` 混用 |
| DJI 反馈 `encoder/velocity_rpm/current_raw/raw` | 原始协议值，保留电机本体方向 |
| DJI `velocity_rad_s/angle_rad/total_angle_rad` | 机构输出轴 rad/s、rad，应用方向和减速比 |
| DJI `current_a` | 按 `init()` 指定的 A/LSB 反馈比例换算并应用方向；默认沿用 SENTRY 的 0.001，须核实固件/实测 |
| LK 速度/位置指令及换算反馈 | 机构输出轴 rad/s、rad；电流 A 按机构方向，原始 `iq_raw/speed_dps/encoder/raw` 保留本体值 |
| DM 位置/速度/力矩 | 机构输出轴 rad、rad/s、N·m；外部减速按理想无损传动换算 |

DJI 默认使用型号减速比，`init()` 的比例传 0 选择型号默认值。
控制任务统一使用位置 rad、角速度 rad/s、角加速度 rad/s²、电流 A、力矩 N·m、周期 s。
这与 SENTRY 的 `motor.h`、`CAN_receive.c` 和云台控制任务的单位一致。
DJI 发送值仍与 SENTRY 的 `set.value` 一样使用原始控制编码，GM6020 输出不是以 V 为单位的电压。
`encoder/velocity_rpm/current_raw` 等明确标注的原始字段只用于协议诊断。
LF 的运动学反馈保留现有机构输出轴定义；SENTRY 的 DJI 接收层使用转子轴，移植时仍需核对减速比。

| DJI 型号 | 默认减速比 | `setCommand()` 输出范围（原始控制编码） |
| --- | --- | --- |
| GM6020 | 1 | 电压量 -25000 ~ 25000 |
| M3508 P19 | 3591/187 ≈ 19.2032 | 电流量 -16384 ~ 16384（指令对应 ±20 A） |
| M2006 P36 | 36 | 电流量 -10000 ~ 10000（指令对应 ±10 A） |

输出上限集中定义在 `dvc_dji_motor.hpp` 的 `DJI_*_MAX_COMMAND` 常量中，驱动与调参台共用。
GM6020 的 ±25000 是本地 20231013 版使用说明规定的电压指令范围，也与 SENTRY 大 yaw 配置一致。
M3508 裸电机、改装减速箱应显式传入实际减速比，不能套用 P19 原装减速比。
本地 DJI 手册只明确了指令电流量程，未明确反馈电流 A/LSB。`init()` 最后一个参数
`feedback_current_a_per_lsb` 可配置反馈比例；默认 0.001 为 SENTRY 兼容约定，不能视为厂家确认值，
也不能从指令的 ±16384→±20 A 或 ±10000→±10 A 直接推定反馈比例。
GM6020 新固件另有电流模式（0x1FE/0x2FE，±16384 对应 ±3 A），本驱动的 GM6020 仍为电压模式。
应用控制器可按机构需要设置更低的上限。直接调用 `sendFrame()` 时由调用方限幅。

LK 手册中的位置、速度指令为电机轴量；有内置减速箱时，比例必须包含该减速箱，
例如协议电机轴转 10 圈、输出轴转 1 圈应传 10。当前无减速 MF 调参配置仍为 1。
DM 的比例按其固件实际协议轴设置，避免重复计入已经在驱动器内换算的减速比。
LK 的编码器零值对应角度 0，单圈角度折返后再按减速比换算；0x9C 不提供累计圈数。
`sendPosition(angle_rad, max_velocity_rad_s)` 的两个参数统一使用机构轴 rad、rad/s，
驱动内部才换成角度制协议编码；例如 `sendPosition(1.57f, 3.141592654f)` 约为 90°、180°/s。
0xA3/0xA4 为多圈位置命令，角度编码为 int32 的 0.01°，0xA4 最大速度编码为 uint16 的 deg/s。
驱动不再把目标裁成 ±360° 或把速度裁成 360 deg/s；编码溢出及小于一个速度量化单位的非零上限
返回 false。机构实际运动范围由应用限位和驱动器 Max Angle/Max Speed 等参数决定。
当前调参台仍保留一圈目标范围，外部 SMC 仍使用单圈反馈；多圈指令支持不等于多圈闭环反馈支持。
`sendSpeed(velocity_rad_s, current_limit_a)` 的电流上限以 A 输入，MF 限幅到 16.5 A，
MG 限幅到 33 A；0 原样发送编码 0，手册未定义其特殊含义，需按固件验证。
非零但小于 1 LSB 的电流上限拒绝发送，避免量化为 0。显式原始接口 `sendTorqueCurrent()` 仍接收 ±2048 编码。
DJI 原 `angle_deg/total_angle_deg` 已改为 `angle_rad/total_angle_rad`，调用方无需再乘角度转弧度系数。

本次核对的手册、页码及保留事项见 [本地手册核对记录](MANUAL_REVIEW.md)。

DM MIT 的 `kp/kd` 也按机构轴定义，传入电机时除以外部减速比的平方。
MIT 有限数值按电机参数范围限幅；NaN/无穷输入拒绝发送。
浮点位置速度/速度模式在变换后超出所配置范围时返回 false。
位置速度模式第二参数为非负速度上限，不因安装方向改变符号。
DM 的每个发送接口必须与 `init()` 选择的模式一致。
本驱动保留原工程的“特殊命令 ID = 基址 + 模式偏移”固件约定；不同固件的特殊命令地址
应按随电机协议核对，不能仅由品牌名认定兼容。

## DJI 共享帧

```cpp
static DjiMotor left, right;
// 应用初始化阶段：实际项目应逐项处理失败。
bool ok = left.init(MotorType::DjiM3508, 1, &hfdcan1, 1)
       && right.init(MotorType::DjiM3508, 2, &hfdcan1, -1)
       && MotorBase::registerMotor(left)
       && MotorBase::registerMotor(right);
// ok 为 true 后再启动控制任务。

// 各控制模块更新指令；输出许可为 false 或反馈离线时主动设置 0。
left.setCommand(1000);
right.setCommand(1000);

// 同一 (CAN 口, 发送 ID) 由一个任务负责周期发送。
// 自动汇总注册表中的 left/right，实际槽位分别为 +1000/-1000。
bool queued = DjiMotor::sendGroup(&hfdcan1, 0x200);
```

`sendGroup()` 不再接受对象数组，避免不同模块漏传其他槽位。
它只汇总已注册的 DJI 对象；应用负责指令更新周期和唯一发送任务。
所有发送 API 返回 true 只表示已写入硬件 FIFO，不能作为电机执行成功的确认。

## 反馈读取与输出许可

```cpp
const auto feedback = motor.data();
const bool online = motor.isOnline();
const uint32_t rx_count = motor.rxCount();
const uint32_t last_rx_tick = motor.lastRxTick();
```

`data()` 在短临界区内返回反馈副本；在线检测、接收计数和接收时间沿用基类接口，
不另建快照结构。确需将这几项一起读取时，调用方可用 `MotorCriticalSection` 包住读取操作。
tick 为 0 的有效反馈同样在线；离线判断支持 uint32_t 节拍回绕。
C610/C620 反馈 DATA[7] 保存在 `error_code` 中，收到有效反馈后非零表示驱动器报错。
C610 DATA[6] 是保留位，`temperature_valid = false`，不能把数值 0 当作实测温度。
GM6020 和 LK 0x9C 不携带故障字段，不能从在线状态推断无故障。
DM 直接读取反馈 `state`：0/1 表示未报告故障，8~15 表示故障，其他为保留状态。
故障帧仍是有效通信；应用按型号解释原始故障字段，驱动不再提供统一故障快照。

应用应结合遥控/上位机在线、急停、温度、驱动器故障和机构限位决定输出许可。
DJI 失去许可后持续发送零控制量；LK/DM 使用对应停止/失能指令，并按反馈有限重试。
应用重新确认控制输入和有效反馈后才能重新使能。底层驱动不擅自设置温度阈值或自动使能。
LK 型号相关的故障读取命令需按该型号协议扩展，不能把 0x9C 的编码器字段当故障字。

## CAN 接收、诊断与恢复

接收遵循 CubeMX 配置：优先使用已分配的 FIFO0，否则使用 FIFO1。
当前 CAN1/2 使用 FIFO0（各 4 个元素），CAN3 使用 FIFO1（4 个 64 字节元素）。
驱动不改写 CubeMX 分配的容量、位时序或帧格式。
电机桥只接收标准经典数据帧，电机解析器进一步要求长度为 8。
其他标准数据帧可通过 `bspFdcanAddObserver()` 添加最多 8 个接收观察者。
观察者在 ISR 同步调用，参数仅在回调期间有效，不得阻塞；需要延后处理时自行复制到队列。

`bspFdcanGetStats()` 返回每路成功入队数、发送失败数、接收帧数、FIFO 丢帧事件数、
错误事件数、Bus-off 次数和恢复次数。硬件丢帧位只能统计事件，不能推算精确丢失帧数。
`AutoRetransmission` 保留关闭：周期命令由下个周期更新，使能等事务由应用确认并有限重试。

Bus-off 锁存后拒绝发送。由应用停止该总线所有发送者，再在任务上下文调用
`bspFdcanRecover()`。恢复重建 HAL 和消息 RAM，清除旧 TX 请求；不会自动发送使能。
成功后对该总线电机调用 `reset()`，重新请求反馈并确认在线后，才恢复应用输出。
恢复失败保持禁止发送。禁止在 ISR 或屏蔽中断时调用恢复接口。
外设配置在 CubeMX 中修改并重新生成；BSP 跟随生成的 FIFO 分配。
主机回归检查配置一致性和消息 RAM 分配，不要求所有总线使用同一个 FIFO。

## 验证

`python tests/run_host_tests.py` 执行算法和遥控器等主机回归。
`tests/run_motor_unit_tests.py` 中的故障断言仍依赖已移除的快照接口，当前不能直接编译；
本次保持 `tests` 原样，在 `build/snapshot_cleanup/motor_units_current_api.cpp` 临时副本中
改为检查 `data()`、`isOnline()` 和原始错误码，电机单位、输出限幅、LK 编码及 SMC 换算均通过。
实际调参模块的故障启动拒绝和运行中故障停机验证也通过。
这些主机检查使用实际电机和 SMC 源码，CAN/HAL 为测试适配器；不替代 MC02 上的波形、
总线负载、驱动器故障与恢复实测。

协议参考：[达妙 SDK](https://github.com/dmBots/motor-sdk)，
[达妙电机说明书](https://f.foxtech.com/manual/DM-J8009-2EC-V1.0.pdf)。
