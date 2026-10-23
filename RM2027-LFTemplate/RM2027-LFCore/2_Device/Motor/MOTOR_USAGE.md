# 电机与 CAN 模板使用约定

模板默认不调用 `lkTestInit()` 或 `smcTuneInit()`，保留两个测试模块的源码和编译检查。
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
| DJI `velocity_rad_s/angle_deg/total_angle_deg` | 机构输出轴反馈，应用方向和减速比 |
| LK 速度/位置指令及换算反馈 | 机构输出轴 rad/s、rad；电流 A 按机构方向，原始 `iq_raw/speed_dps/encoder/raw` 保留本体值 |
| DM 位置/速度/力矩 | 机构输出轴 rad、rad/s、N·m；外部减速按理想无损传动换算 |

DJI 默认使用型号减速比，`init()` 的比例传 0 选择型号默认值。
LK、DM 协议本身通常已报告自带减速器的输出轴，默认比例为 1，不能再次乘内置减速比。
LK 的编码器零值对应角度 0，单圈角度折返后再按外部减速比换算；没有累计圈数。
LK 位置指令保留原驱动的协议轴 ±360° 限幅，速度上限为协议轴 360 deg/s。

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

## 反馈快照与输出许可

```cpp
const auto feedback = motor.readSnapshot();
// feedback.data、last_rx_tick、rx_count、online 在同一短临界区复制。
// feedback.fault_known == false 时，不能从 fault == false 推断电机无故障。
```

`data()` 返回独立副本；需要反馈和元信息保持一致时用 `readSnapshot()`。
tick 为 0 的有效反馈同样在线；离线判断支持 uint32_t 节拍回绕。
DJI 原始反馈和 LK 0x9C 不携带完整故障字，因此快照标记 `fault_known = false`。
DM 反馈状态 0/1 表示未报告故障，8~15 表示故障，其他保留状态标记未知。
故障状态仍是有效通信，`online` 与 `fault` 独立。

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

运行 `python tests/run_motor_tests.py` 单独检查电机/CAN，或运行
`python tests/run_host_tests.py` 执行全部主机回归。
主机测试使用实际电机和 BSP 源码，HAL 仅为测试适配器；不替代 MC02 上的波形、
总线负载、驱动器故障与恢复实测。

协议参考：[达妙 SDK](https://github.com/dmBots/motor-sdk)，
[达妙电机说明书](https://f.foxtech.com/manual/DM-J8009-2EC-V1.0.pdf)。
