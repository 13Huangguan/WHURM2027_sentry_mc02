# 裁判系统模块修改日志

**协议基准**：《RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）》
路径：`D:\Astudy\RM\裁判系统\RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）.pdf`

原代码基于 **1.6.1**，有 7 个 cmd_id 的结构体与 2026 版对不上。本次全部对齐。

---

## 1. 解析层修正

### 1.1 结构体尺寸对齐（协议表 1-6 ~ 表 1-25）

| cmd_id | 原来 | 现在 | 说明 |
|---|---|---|---|
| `0x0003` | 16 B | **20 B** | `reserved` 实为 `int16 damage_difference`；补 `enemy_outpost_HP` / `enemy_base_HP` |
| `0x0201` | 13 B | **17 B** | 补 `float bullet_speed_limit`（offset 12） |
| `0x0204` | 6 B | **8 B** | `cooling_buff` 是 `uint16_t` 不是 `uint8_t`，原来从它开始全部错位；补 `remaining_energy` |
| `0x020D` | 6 B | **14 B** | 补 `uint64_t sentry_info_3`（六种姿态弱化剩余时长） |
| `0x0301` | 119 B | **118 B** | `user_data[113]` → `[112]` |
| `0x020B` | 40 B | 40 B | 布局本就对，`standard_5_x/y` 名字错，实为保留位 |
| `0x0202` | 14 B | 14 B | 布局对，但**前 8 字节已是保留位**，见 1.4 |

尺寸已用 `static_assert` 逐个核对协议表，全部一致。

### 1.2 删除已废弃结构体

`0x0102` 补给站动作、`0x0103` 补给站预约、`0x0205` 空中机器人能量、`0x0304` 机器人指令 —— **V2.0.0 中已删除**。
`0x020C` 雷达标记进度改成单个 `uint16_t`，且只发给雷达机器人，哨兵收不到，一并删除。

对应 `protocol.h` 的 cmd_id 枚举、`referee.cpp` 的 `DataSolve()` case、`Init()` 的 `memset` 全部同步清理。

### 1.3 修掉的 bug

- **`0x020B` 只拷了 12 字节**：`memcpy(&ground_robot_pos_t, ..., sizeof(robot_pos_t))` —— 目标是 40 字节的 `ground_robot_position_t`，后 7 个 float 永远是 0。
- **`Init()` 漏 memset**：`RFID_STATUS` / `ground_robot_pos_t` / `sentry_decision_info_t` / `unpack_obj` 没有清零，解包状态机可能从随机状态启动。

### 1.4 ⚠️ 底盘功率没了

`0x0202` 的前 8 字节在 V2.0.0 中全部改为保留位。**2026 赛季裁判系统不再下发实际底盘功率**，只给缓冲能量。

原 `power_heat_data.chassis_voltage` / `chassis_current` / `chassis_power` 已改名为 `reserved1/2/3`，`GetChassisPower()` / `GetChassisPowerAndBuffer()`（全工程零调用）已删除。**功率控制需自行采样。**

### 1.5 ID 编号修正

`robot_id_t` 蓝方从 `11~17` 改为 `101~111`（见协议附录二），补全飞镖/雷达/基地/前哨站，并加 `REFEREE_SERVER = 0x8080`（哨兵自主决策指令的接收者）。

---

## 2. USB 发送补齐

**这一层的结构体是板子↔上位机的约定，改动需上位机同步。**

| 包 | 改动 |
|---|---|
| EVENT `0x04` | 3 字节自定义位域 → `uint32_t event_data` 原样透传协议 0x0101 的原始值 |
| ALL_ROBOT_HP `0x06` | 补 `damage_difference` / `enemy_outpost_HP` / `enemy_base_HP` |
| GROUND_ROBOT_POSITION `0x09` | 10 个 float 全部填上（原来整包为空） |
| ROBOT_STATUS `0x0B` | 发送字段 5 → 17 个；`current_up` 拼写修正为 `current_hp` |
| BUFF `0x0D` | `cooling_buff` u8→u16；6 个字段全部填上（原来整包为空） |
| SENTRYINFO `0x0E` | 补 `uint64_t sentry_info_3` |
| ROBOT_STATE_INFO `0x03` | 补 `color`（`GetTeamColor()`）、`attacked`（新增 `IsAttacked()`） |
| JOINT_STATE `0x0C` | **已注释** —— 非裁判数据，`CmdGimbalJointState()` 全工程无定义 |

**包尺寸变化**（上位机解析代码必须同步改）：

| 包 | 变化 |
|---|---|
| ALL_ROBOT_HP `0x06` | +4 B |
| EVENT `0x04` | +1 B |
| BUFF `0x0D` | +1 B |
| SENTRYINFO `0x0E` | +8 B |
| ROBOT_STATUS `0x0B` | +约 20 B |

新增 `Referee::IsAttacked()`：用血量差值锁存 1 s 判定"正在挨打"。不用 `0x0206` 的 `HP_deduction_reason`，因为 reason 0 是合法值（被弹丸击打），无法区分"没挨打"和"被弹丸打"。

---

## 3. 工程接线

1. **`Core/Src/freertos.c` 注册了 `usb_task`**（按 `referee_task` 的写法）。原来完全没注册，`--gc-sections` 把它回收掉了，`usb.Update()` 从未执行。
2. **`MX_USB_DEVICE_Init()` 原先只在 `__weak INS_Task` 桩里**，而该桩被 `app_ins.cpp` 的强符号覆盖 —— **USB 设备从未初始化过**。现由 `usb_task` 自己调用。
3. **`usb_typdef.h` 补了构型宏**：工程没有 `robot_param.h` / `robot_typedef.h`，先在那里给出带 `#ifndef` 保护的默认值。当前填的是：

```c
#define CHASSIS_TYPE        CHASSIS_OMNI_WHEEL
#define GIMBAL_TYPE         GIMBAL_YAW_PITCH_DIRECT
#define SHOOT_TYPE          SHOOT_FRIC_TRIGGER
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE
```
依据是 `usb.cpp` 里的 `C_BOARD_OMNI_SENTRY_ROT_GIMBAL`。**若实际是麦轮或双 Yaw，改这 4 行。**

---

## 4. 仍未发送

### 有包但恒为 0（均与裁判系统无关）

| 包 | 原因 |
|---|---|
| DEBUG `0x01` | `ModifyDebugData()` 无人调用 |
| ROBOT_MOTION `0x08` | 读 `system_state.Read().chassis`，但 `UpdateChassis()` 全局无调用点 |
| JOINT_STATE `0x0C` | 等云台模块 |

### 裁判收了但没有 USB 包

| cmd_id | 数据 | 建议 |
|---|---|---|
| `0x0104` | `referee_warning` 裁判警告 | **建议加**，哨兵吃牌要知道 |
| `0x0002` | `game_result` 比赛结果 | 可选 |
| `0x0105` | `dart_info` 飞镖 | 哨兵用不上 |
| `0x020A` | `dart_client_cmd_t` | 哨兵用不上 |
| `0x020E` | `radar_info_t` | 只发雷达 |
| `0x0302` | `CUSTOM_CONTROLLER_DATA` | 走图传，哨兵没有 |

新增需在 `usb_typdef.h` 加 struct + 帧 ID + `Init()` 初始化 + `SendXxx()`，**且上位机必须同步改**。

---

## 5. 快速参考

| 想改什么 | 去哪 |
|---|---|
| 本车构型 | `usb_typdef.h` 的 `CHASSIS_TYPE` 等 4 个宏 |
| 新增裁判数据解析 | `protocol.h` 加 cmd_id → `referee.hpp` 加结构体 → `referee.cpp` 的 `DataSolve()` 加 case |
| 新增 USB 发送 | `usb_typdef.h` 加 struct+帧 ID → `usb.hpp` 加成员+函数声明 → `usb.cpp` 的 `Init()`/`SendAll()`/`SendXxx()` |
| 发送频率 | `usb.cpp` 顶部的 `SEND_DURATION_xxx` |
| 裁判离线判定 | `referee.hpp` 的 `kTimeout`（300 ms） |

---

## 6. 验证状态

- ✅ `make` 构建通过，`usb_task` / `referee_task` / `USART1_IRQHandler` 均保持 C 链接
- ✅ 22 个协议结构体尺寸 `static_assert` 全部匹配协议表
- ❌ **未上机验证** —— 需接真实裁判系统，用 `RM2027-LFCore/4_Application/Referee/tools/referee_test_tool.py` 或整机联调确认
