# INS 模块修改日志


> ##两个尚未在实机验证的点
>
> **1. 姿态解算算法选型未定。** 工程里同时保留了 **QuaternionEKF** 和 **Mahony** 两套解算，用一个宏（`INS_USE_MAHONY`）切换。**两套都还没在机器人上做过对比测试**，当前默认 EKF 只是为了让改动最小、行为可预期。最终用哪一套要看实测数据 —— 详见 [第 10 节](#10-ekf-vs-mahony对比与选型)，里面有完整的对比表和测试方案。
>
> **2. 陀螺零偏值是手工测量后写定。** 测量方法见下文（[第 3 节](#3-暖机零偏标定新增功能当前关闭)有标定功能）。
>

---

## 1. BMI088 陀螺带宽

**文件**：`RM2027-LFCore/2_Device/BMI088/Bmi088.c:90`

```c
// 原来
{BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},

// 现在
{BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},
```
**副作用**：ODR 同时从 2000 降到 1000 Hz。固件读取速率约 943 Hz（1 ms 延时 + 循环体耗时），**略低于 ODR，不会读到重复样本**，每个读到的都是新鲜样本。之前 ODR 2000 Hz 时是 2:1 过采样，两者都可以。

---

## 2. 姿态解算：EKF ↔ Mahony 可切换

**新增**：`RM2027-LFCore/1_Middleware/Algorithm/Mahony/alg_mahony.c` / `.h`
**文件**：`RM2027-LFCore/4_Application/INS/app_ins.cpp:26-38`

```c
#ifndef INS_USE_MAHONY
#define INS_USE_MAHONY  0        // ← 0 = EKF（当前）  1 = Mahony
#endif

#if INS_USE_MAHONY
#include "alg_mahony.h"
#else
#include "Quaternion.h"
#endif
```

>  **提示**：Mahony 和 EKF 一样**无法观测 Yaw**。三轴加速度计只能确定倾斜的 2 个自由度，绕重力轴的旋转不可观测，所以 Yaw 零偏**无法在线估计**。EKF 的 `Quaternion.c:160` 直接把 `Deviate[2] = 0.f` 写死就是这个原因。
>

---


**作用**：静止时把小于 0.69 °/s 的角速度当噪声清零，抑制 Yaw 随机游走。位置在减零偏之后、姿态解算之前，两套滤波器都吃这个处理。

**代价**：**低于 0.69 °/s 的真实旋转也会被抹掉**，云台无法跟踪这么慢的转动。要更灵敏就把值调小（如 `0.005f`），但要接受更多漂移。

**需要采集的数据**（建议用 emberprobe 记录，和之前一样）：

| # | 测试项 | 采集变量 | 判定指标 |
|---|---|---|---|
| 1 | **静止 Yaw 漂移** | `INS_Info.Yaw_Angle`，采 3 分钟 | 漂移速率（°/min）+ 随机游走幅度 |
| 2 | **静止 Roll/Pitch 噪声** | `INS_Info.Pitch_Angle` / `Roll_Angle`，3 分钟 | 标准差 |
| 3 | **线性加速度干扰** | 推着机器人平移，记录 roll/pitch | 被带偏的最大幅度 + 回正时间 |
| 4 | **快速转动收敛** | 手动快速转 yaw/pitch 后静止 | 回正速度 + 有无振荡 |
| 5 | **EKF 门控效果**（可选） | 临时把卡方阈值调大 | 平移时的表现差异 |

**判定规则：**

1. **如果第 1 项两者接近** —— 这是预期结果，因为 Yaw 不可观测，两边应该差不多。若确实如此，说明滤波器选择对 Yaw 没有影响，只看 2~4 项。
2. **如果平移时 EKF 明显更稳**（第 3 项被带偏更小）—— 选 **EKF**
3. **如果两者接近** —— 选 **Mahony**，因为参数少、好调、好维护
4. **如果 Mahony 的 roll/pitch 更抖** —— 先试把 `Mahony_Kp` 从 2.0 调小到 1.0 再比，不要直接下结论

**记录方式**：两次测试各存一份 CSV，文件名带上算法名（如 `ekf_static_3min.csv` / `mahony_static_3min.csv`），方便回看对比。

### 切换方法

```c
// app_ins.cpp:31
#define INS_USE_MAHONY  0      // 0 = EKF    1 = Mahony
```

改一个数、重新编译即可。两套算法的输入完全一致（同样的陀螺读数、同样的死区、同样的二阶低通后的加速度计），输出也都写进 `INS_Info.Angle[]`，**对下游代码完全透明**，切换不影响任何其他模块。

---

---

## 3. 暖机零偏标定（新增功能，**当前关闭**）

**文件**：`app_ins.cpp`，由 `BMI088_Warm_Calib_Enable` 开关控制

```c
#define BMI088_Warm_Calib_Enable  0     // ← 需要用的时候改成 1
```

**用途**：在 40°C 工作温度下测出准确的陀螺零偏，然后写进 `Bmi088.c` 的 `#else` 分支，把 `IMU_Calibration_ENABLE` 设成 `0U`。

**工作方式**：

| 参数 | 值 | 含义 |
|---|---|---|
| `Temp_Band` | 0.02 °C | 温度读数在这个带内才采样本 |
| `Window_Samples` | 5000 | 每个窗口的样本数 |
| `Window_Num` | **500** | 窗口数见下 |

温度读数是 0.125°C 量化，所以 `|T-40| < 0.02` 实际等价于**"读数恰好是 40.000 时才采"** —— 暖机瞬态被门限自动滤掉，不需要额外的稳定等待。

**使用方法**：

1. `BMI088_Warm_Calib_Enable` 改成 `1`，编译烧录
2. 机器人**放平静止**，上电
3. 调试器里 watch 三个变量：
   - `Warm_Calib_State` —— `0` 还没采到样本 / `1` 采集中 / `2` 完成
   - `Warm_Count` —— 已完成窗口数，用来看进度
   - `Warm_Gyro_Offset_Average[0..2]` —— **最终结果**（`Warm_Gyro_Offset` 只是最新单个窗口的值，别读错）
4. 等 `Warm_Calib_State` 变成 `2`，把 `Warm_Gyro_Offset_Average[0..2]` 填进 `Bmi088.c` 的 `#else` 分支
5. `IMU_Calibration_ENABLE` 设成 `0U`，`BMI088_Warm_Calib_Enable` 改回 `0`





---


---


## 4. 快速参考：常见修改点在哪

| 想改什么 | 改哪里 |
|---|---|
| 切换姿态算法（EKF / Mahony） | `app_ins.cpp:31` `INS_USE_MAHONY` |
| Yaw 死区阈值 | `app_ins.cpp:88` `BMI088_GYRO_Z_DEADBAND` |
| 目标温度 | `app_ins.cpp:86` `BMI088_Target_Temperature` |
| 加热功率上限 | `app_ins.cpp:90` `BMI088_Heater_PWM_Max` |
| 温控 PID | `app_ins.cpp` `INS_Task_Init()` 里的 `TempCtrl_PID.init(...)` |
| 暖机标定开关 | `app_ins.cpp:94` `BMI088_Warm_Calib_Enable` |
| 陀螺带宽 | `Bmi088.c:90` |
| 陀螺零偏 | `Bmi088.c` `#else` 分支 + `IMU_Calibration_ENABLE` |
---

