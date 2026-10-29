# 全向底盘运动补偿算法

本模块实现《全向底盘运动补偿》文档中的三段处理，并已接入底盘控制任务：

1. 四轮轮速逆解，得到车体 `vx/vy`。
2. 使用 `[vx, ax, vy, ay]` 恒加速度卡尔曼滤波融合轮速和 IMU 加速度。
3. 可选的 `Kp = K * |yaw_rate|` 目标速度补偿。

## 输入要求

- 四个轮速为逻辑轮序 1~4 的轮缘线速度，单位 `m/s`。
- 轮位图至少有三轮有效才会进行轮速逆解。
- Yaw 角必须是期望/云台坐标系相对底盘坐标系的相对角，单位 `rad`。
- 加速度必须已经转到**底盘平面坐标系**并去除重力，单位 `m/s²`。
- 轮子打滑判断由调用方完成。当前底盘仅在四轮均在线时提供全有效位图。
- 底盘使用融合后的 `vx/vy` 作为速度环反馈，并应用受限的旋转目标补偿。
- IMU 加速度无效或融合尚未建立时，底盘自动退回轮速一阶低通。

## 使用示例

```cpp
#include "alg_motion_compensation.hpp"

static motion_compensation::OmniMotionCompensator compensator;

void Init()
{
    compensator.init();  // 默认参数仅用于初步调试
}

void Update1ms()
{
    motion_compensation::Measurement measurement {};
    measurement.wheel_linear_velocity_mps[0] = wheel_linear[0];
    measurement.wheel_linear_velocity_mps[1] = wheel_linear[1];
    measurement.wheel_linear_velocity_mps[2] = wheel_linear[2];
    measurement.wheel_linear_velocity_mps[3] = wheel_linear[3];
    measurement.valid_wheel_mask = motion_compensation::kAllWheelsValidMask;

    measurement.yaw_rad = relative_yaw_rad;
    measurement.yaw_rate_rad_s = relative_yaw_rate_rad_s;
    measurement.acceleration_x_mps2 = chassis_ax_mps2;
    measurement.acceleration_y_mps2 = chassis_ay_mps2;
    measurement.acceleration_valid = imu_acceleration_valid;
    measurement.dt_s = 0.001f;

    (void)compensator.update(measurement);

    const auto estimate = compensator.estimate();
    const auto corrected = compensator.compensate(target_vx_mps, target_vy_mps);
    target_vx_mps = corrected.vx_mps;
    target_vy_mps = corrected.vy_mps;
}
```

## 接入底盘前的验证

- 确认四个轮速顺序与底盘逆解矩阵一致。
- 确认 Yaw 角的正负和零位。
- 确认 BMI088 三轴方向；`app_ins` 会先在 g 域去除重力，再换算为 `m/s²`
  发布给底盘，换板或改变 IMU 安装方向后必须重新核对正负号。
- 用实车分别测试仅轮速、轮速加加速度、再加入目标补偿三组效果。
- 若需要允许电机打滑，必须接入外部打滑检测并填写 `valid_wheel_mask`。

