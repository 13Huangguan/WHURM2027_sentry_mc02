#include "app_chassis.h"

#include "alg_basic.hpp"
#include "alg_force.hpp"
#include "alg_pid.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "dvc_dji_motor.hpp"
#include "lf_core.hpp"
#include "remote_control.h"

namespace {

constexpr uint8_t kWheelCount = 4U;

// 逻辑轮序与 ForceSolver/buildOmniDistribution 一致。
// 如果物理接线或安装方向不同，优先修改这两个数组。
constexpr uint8_t kWheelMotorId[kWheelCount] = {2U, 3U, 4U, 1U};
constexpr int8_t kWheelDirection[kWheelCount] = {1, 1, -1, -1};

constexpr uint32_t kChassisTxId = 0x200U;
constexpr uint32_t kOfflineTimeoutMs = 10U;

// 参考26哨兵旧底盘参数。
constexpr float kWheelRadius = 0.050f;
constexpr float kWheelCenterDistance = 0.250f;
constexpr float kWheelGearRatio = 19.0f;

// ForceSolver 的力矩是轮子输出轴力矩。
// ratio 设为 1，TorqueConstant 使用输出轴 N*m/A，
// 因此 getOut() 直接得到电机电流 A。
constexpr float kWheelTorquePerAmp = 0.150f;
constexpr float kWheelTorqueMax = 2.500f;
constexpr float kC620MaxCurrent = 20.0f;

constexpr float kMaxVx = 1.0f;
constexpr float kMaxVy = 1.0f;
constexpr float kMaxWz = 2.0f;

constexpr float kMaxFx = 70.0f;
constexpr float kMaxFy = 70.0f;
constexpr float kMaxMz = 12.0f;

constexpr float kRcDeadzone = 0.05f;
constexpr float kRcXSign = 1.0f;
constexpr float kRcYSign = -1.0f;
constexpr float kRcWzSign = -1.0f;

constexpr float kVelocityFilterAlpha = 0.75f;

DjiMotor g_wheelMotor[kWheelCount];
ForceSolver g_chassisForce;
PID g_vxPid;
PID g_vyPid;
PID g_wzPid;

float g_vxFiltered = 0.0f;
float g_vyFiltered = 0.0f;
float g_wzFiltered = 0.0f;
bool g_forceReady = false;

void PublishChassisVelocity(void) {
    LuojiaFox::system_state.updateChassis({
        g_vxFiltered,
        g_vyFiltered,
        g_wzFiltered
    });
}

float ApplyDeadzone(float value) {
    return (basic_math::abs(value) < kRcDeadzone) ? 0.0f : value;
}

void ResetVelocityPids(void) {
    g_vxPid.setIntegralError(0.0f);
    g_vyPid.setIntegralError(0.0f);
    g_wzPid.setIntegralError(0.0f);
}

bool AllWheelsOnline(void) {
    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        if (!g_wheelMotor[i].isOnline(now)) {
            return false;
        }
    }
    return true;
}

void SendCurrentCommand(const float command[kWheelCount]) {
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        float raw = command[i];
        raw = basic_math::constrain(raw, -16384.0f, 16384.0f);
        g_wheelMotor[i].setCommand(static_cast<int16_t>(raw));
    }

    (void)DjiMotor::sendGroup(&hfdcan1, kChassisTxId);
}

void SendZeroCurrent(void) {
    const float zero[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    SendCurrentCommand(zero);
}

void EstimateBodyVelocity(
    const float wheelOmega[kWheelCount],
    float *vx,
    float *vy,
    float *wz
) {
    constexpr float c[kWheelCount] = {
        0.70710678f, 0.70710678f,
        -0.70710678f, -0.70710678f
    };

    constexpr float d[kWheelCount] = {
        -0.70710678f, 0.70710678f,
        0.70710678f, -0.70710678f
    };

    float wheelLinear[kWheelCount];

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        wheelLinear[i] = kWheelRadius * wheelOmega[i];
    }

    *vx = 0.5f * (
        c[0] * wheelLinear[0] + c[1] * wheelLinear[1] +
        c[2] * wheelLinear[2] + c[3] * wheelLinear[3]
    );

    *vy = 0.5f * (
        d[0] * wheelLinear[0] + d[1] * wheelLinear[1] +
        d[2] * wheelLinear[2] + d[3] * wheelLinear[3]
    );

    *wz = -(
        wheelLinear[0] + wheelLinear[1] +
        wheelLinear[2] + wheelLinear[3]
    ) / (4.0f * kWheelCenterDistance);
}

}  // namespace

void Chassis_Init(void) {
    ForceActuatorParam params[kWheelCount] = {};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const uint32_t motorId = kWheelMotorId[i];

        if (!g_wheelMotor[i].init(
                MotorType::DjiM3508,
                static_cast<uint8_t>(motorId),
                &hfdcan1,
                kWheelDirection[i],
                kWheelGearRatio
            ) ||
            !MotorBase::registerMotor(g_wheelMotor[i])) {
            Error_Handler();
        }
        g_wheelMotor[i].setOfflineTimeout(kOfflineTimeoutMs);

        params[i].torque_constant = kWheelTorquePerAmp;
        params[i].reduction_ratio = 1.0f;
        // 电机方向由 DjiMotor 统一处理，力控输出使用机构输出轴坐标。
        params[i].direction = 1;
        params[i].torque_max = kWheelTorqueMax;

        // 调试阶段先关闭模型补偿。
        params[i].inertia = 0.0f;
        params[i].viscous_friction = 0.0f;
        params[i].coulomb_friction = 0.0f;
        params[i].gravity_amplitude = 0.0f;
        params[i].gravity_offset = 0.0f;
    }

    g_chassisForce.init(params, kWheelCount, 3U, 0.001f);
    g_chassisForce.setVelocityFilterAlpha(0.8f);

    float distribution[kWheelCount * 3U] = {};

    g_forceReady = ForceSolver::buildOmniDistribution(
        kWheelRadius,
        kWheelCenterDistance,
        distribution
    );

    if (g_forceReady) {
        g_chassisForce.setDistribution(distribution);
    }

    g_vxPid.init(
        25.0f, 5.0f, 0.0f, 0.0f,
        20.0f, kMaxFx, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    g_vyPid.init(
        25.0f, 5.0f, 0.0f, 0.0f,
        20.0f, kMaxFy, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    g_wzPid.init(
        4.0f, 0.5f, 0.0f, 0.0f,
        4.0f, kMaxMz, 0.001f,
        0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable
    );

    g_vxFiltered = 0.0f;
    g_vyFiltered = 0.0f;
    g_wzFiltered = 0.0f;

    SendZeroCurrent();
}

void Chassis_Control(void) {
    float wheelOmega[kWheelCount];

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        wheelOmega[i] = g_wheelMotor[i].data().velocity_rad_s;
    }

    float vxRaw = 0.0f;
    float vyRaw = 0.0f;
    float wzRaw = 0.0f;
    EstimateBodyVelocity(wheelOmega, &vxRaw, &vyRaw, &wzRaw);

    g_vxFiltered = kVelocityFilterAlpha * g_vxFiltered +
                   (1.0f - kVelocityFilterAlpha) * vxRaw;

    g_vyFiltered = kVelocityFilterAlpha * g_vyFiltered +
                   (1.0f - kVelocityFilterAlpha) * vyRaw;

    g_wzFiltered = kVelocityFilterAlpha * g_wzFiltered +
                   (1.0f - kVelocityFilterAlpha) * wzRaw;

    PublishChassisVelocity();

    const bool rcOnline = !getRcOffline();
    const bool locked =
        SWITCH_IS_DOWN(getDt7RcSw(RC_SW_RIGHT));

    if ((!rcOnline) || locked ||
        (!AllWheelsOnline()) || (!g_forceReady)) {
        g_vxFiltered = vxRaw;
        g_vyFiltered = vyRaw;
        g_wzFiltered = wzRaw;
        PublishChassisVelocity();

        ResetVelocityPids();
        g_chassisForce.setEnable(false);
        SendZeroCurrent();
        return;
    }

    const float forward = ApplyDeadzone(
        getDt7RcCh(RC_CH_LEFT_VERTICAL)
    );
    const float lateral = ApplyDeadzone(
        getDt7RcCh(RC_CH_LEFT_HORIZONTAL)
    );
    const float rotate = ApplyDeadzone(
        getDt7RcCh(RC_CH_RIGHT_HORIZONTAL)
    );

    g_vxPid.setTarget(kRcXSign * forward * kMaxVx);
    g_vyPid.setTarget(kRcYSign * lateral * kMaxVy);
    g_wzPid.setTarget(kRcWzSign * rotate * kMaxWz);

    g_vxPid.setNow(g_vxFiltered);
    g_vyPid.setNow(g_vyFiltered);
    g_wzPid.setNow(g_wzFiltered);

    g_vxPid.timCalculatePeriodElapsedCallback();
    g_vyPid.timCalculatePeriodElapsedCallback();
    g_wzPid.timCalculatePeriodElapsedCallback();

    const float generalizedForce[3] = {
        basic_math::constrain(g_vxPid.getOut(), -kMaxFx, kMaxFx),
        basic_math::constrain(g_vyPid.getOut(), -kMaxFy, kMaxFy),
        basic_math::constrain(g_wzPid.getOut(), -kMaxMz, kMaxMz)
    };

    g_chassisForce.setTarget(generalizedForce);
    g_chassisForce.setNow(nullptr, wheelOmega);
    g_chassisForce.setEnable(true);
    g_chassisForce.timCalculatePeriodElapsedCallback();

    constexpr float currentToCommand =
        16384.0f / kC620MaxCurrent;
    float currentCommand[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        currentCommand[i] =
            g_chassisForce.getOut(i) * currentToCommand;
    }

    SendCurrentCommand(currentCommand);
}

extern "C" void Control_Task(void const *argument) {
    (void)argument;

    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
