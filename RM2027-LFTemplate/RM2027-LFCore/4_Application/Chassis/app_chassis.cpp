/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_chassis.cpp
  * @brief          : 四轮全向底盘应用实现
  * @author         : Xiaoyuhaoran
  * @date           : 2026/10/29
  * @version        : v2.0
  ******************************************************************************
  * @attention      : 本文件负责底盘电机初始化、遥控速度闭环、力控分配和状态发布。
  *                    Chassis_Init() 之前必须完成三路 FDCAN 初始化。
  *                    电机反馈由 DjiMotor 和 MotorBase 的统一 CAN 分发自动处理。
  ******************************************************************************
  */
/* USER CODE END Header */

/*
 * ============================== 调试变量速查 ==============================
 *
 * 以下变量均为文件级全局变量，可直接加入调试器的 Live Watch。
 *
 * 电机与控制器对象：
 * - g_wheelMotor[4]
 *   四轮 M3508 对象，数组顺序为逻辑轮序 1~4。
 *   可展开 data() 查看角速度、角度、反馈电流、温度和驱动器故障码。
 *
 * - g_chassisForce
 *   四轮力分配与执行器补偿求解器。
 *   getTorque(i) 为轮输出轴力矩，N*m；getOut(i) 为目标电流，A。
 *
 * - g_vxPid / g_vyPid / g_wzPid
 *   vx/vy/wz 三个速度环。getTarget() 为 m/s 或 rad/s，
 *   getOut() 为对应的广义力，N 或 N*m。
 *
 * - g_motionCompensator
 *   轮速与 IMU 加速度融合器。可展开 estimate() 查看当前 vx/vy、ax/ay。
 *
 * - g_powerController
 *   四轮功率衰减器。记录当前预算、衰减前后模型功率及每个电机的输出电流。
 *
 * 功率控制快照：
 * - g_effectivePowerLimitW
 *   当前生效的底盘功率上限，W。一般来自裁判系统，并可能被超级电容进一步限制。
 *
 * - g_estimatedPowerW
 *   功率衰减前的四轮模型总功率，W。
 *
 * - g_consumedPowerW
 *   功率衰减后的四轮模型总功率，W。正常应不大于当前可用预算。
 *
 * - g_powerStatus
 *   Disabled              功率上限为 0，底盘应停止输出
 *   Ready                 未触发功率衰减
 *   Limited               已触发功率衰减且结果满足预算
 *   BudgetInfeasible      当前模型下无法满足预算，本文件会立即停机
 *   InvalidInput          输入或模型非法，本文件会立即停机
 *
 * 速度与运动补偿快照：
 * - g_vxFiltered / g_vyFiltered
 *   PID 当前使用的车体速度反馈，m/s。优先来自运动补偿融合，
 *   融合不可用时退回轮速一阶低通。
 *
 * - g_wzFiltered
 *   PID 当前使用的旋转角速度反馈，rad/s。
 *
 * - g_motionReady
 *   运动补偿器是否初始化成功。false 时始终使用轮速低通。
 *
 * - g_motionUsingAcceleration
 *   当前周期是否使用了新鲜、有效的 IMU 线性加速度。
 *
 * - g_motionAccelerationX / g_motionAccelerationY
 *   运动补偿器估计的底盘 x/y 向加速度，m/s²。
 *
 * 底盘状态与控制源：
 * - g_forceReady
 *   四轮力分配矩阵是否构建成功。false 时禁止底盘输出。
 *
 * - g_chassisMode
 *   当前底盘模式：上位机自动、直接遥控或禁用。
 *
 * - g_modeTransitionPending
 *   模式切换后的首周期停车标志。正常情况下仅短暂为 true。
 *
 * - g_commandSourceReady
 *   当前控制源是否在线、数据新鲜且数值有效。
 *
 * - g_autoCommandLastUpdateMs
 *   最近一次上位机底盘命令的时间戳，单位 ms，对应 HAL_GetTick()。
 *
 * 调参常量位于本注释下方，变量定义处同时标注了单位和作用。
 * ==========================================================================
 */

#include "app_chassis.h"

// 数学、力控分配、PID、CAN、电机、系统状态和遥控器接口。
#include "alg_basic.hpp"
#include "alg_force.hpp"
#include "alg_motion_compensation.hpp"
#include "alg_pid.h"
#include "alg_power_control.hpp"
#include "app_supercap.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "dvc_dji_motor.hpp"
#include "lf_core.hpp"
#include "referee.hpp"
#include "remote_control.h"

#include <cmath>

namespace {

// 当前底盘固定为四轮全向轮结构。
constexpr uint8_t kWheelCount = 4U;

// 逻辑轮序与 ForceSolver/buildOmniDistribution 的矩阵行顺序一致。
// 电机 ID 和方向必须按实车接线校准；修改顺序会影响力控分配结果。
constexpr uint8_t kWheelMotorId[kWheelCount] = {2U, 3U, 4U, 1U};
constexpr int8_t kWheelDirection[kWheelCount] = {1, 1, -1, -1};

// M3508/M2006 ID1~4 共用 0x200 电流控制帧。
constexpr uint32_t kChassisTxId = 0x200U;

// 轮电机连续超过该时间没有有效反馈时，底盘立即停止输出。
constexpr uint32_t kOfflineTimeoutMs = 10U;

// 底盘机械参数，当前沿用 26 哨兵旧底盘的调试值。
constexpr float kWheelRadius = 0.050f;          // 轮半径，m
constexpr float kWheelCenterDistance = 0.250f;  // 轮中心到车体中心的距离，m


// ForceSolver 的力矩是轮子输出轴力矩。
// ratio 设为 1，TorqueConstant 使用输出轴 N*m/A，
// 因此 getOut() 直接得到电机电流 A。
constexpr float kWheelTorquePerAmp = 0.150f;  // 轮输出轴 N*m/A
constexpr float kWheelTorqueMax = 2.500f;     // 单轮输出轴力矩上限，N*m
constexpr float kC620MaxCurrent = 20.0f;      // C620 满量程电流，A

// 裁判数据不可用时用于台架调试的保守上限，对应半自动哨兵基础功率。
constexpr float kFallbackChassisPowerLimitW = 60.0f;

// 规则允许短暂使用缓冲能量；控制侧保留 10% 裕量，避免模型误差持续消耗缓冲。
constexpr float kPowerSafetyMargin = 0.90f;

// 遥控摇杆满量程对应的底盘目标速度。
constexpr float kMaxVx = 1.0f;  // 最大前后速度，m/s
constexpr float kMaxVy = 1.0f;  // 最大横移速度，m/s
constexpr float kMaxWz = 2.0f;  // 最大旋转角速度，rad/s

// 速度环输出经过力控分配前允许的最大广义力。
constexpr float kMaxFx = 70.0f;  // 前后方向合力上限，N
constexpr float kMaxFy = 70.0f;  // 横移方向合力上限，N
constexpr float kMaxMz = 12.0f;  // 旋转力矩上限，N*m

// 遥控器输入处理和车体坐标方向修正。
constexpr float kRcDeadzone = 0.05f;  // 归一化摇杆死区
constexpr float kRcXSign = 1.0f;      // 前后方向符号
constexpr float kRcYSign = -1.0f;     // 横移方向符号
constexpr float kRcWzSign = -1.0f;    // 旋转方向符号

// 运动补偿不可用时，车体速度回退一阶低通的平滑系数。
constexpr float kVelocityFilterAlpha = 0.75f;

// Motion_Compensation 状态估计和旋转目标补偿。
constexpr float kMotionControlPeriodS = 0.001f;
constexpr bool kEnableMotionTargetCompensation = true;
constexpr uint32_t kImuDataTimeoutMs = 10U;

// USB 任务以 1 ms 周期运行，上位机视觉命令超过该时间未更新时停车。
constexpr uint32_t kAutoCommandTimeoutMs = 100U;

// 电机对象必须具有静态生命周期，注册后其地址会保存在 MotorBase 注册表中。
DjiMotor g_wheelMotor[kWheelCount];

// 将三个车体方向的速度误差转换为四个轮子的输出力矩。
ForceSolver g_chassisForce;

// 车体 vx、vy、wz 三个方向的速度闭环。
PID g_vxPid;
PID g_vyPid;
PID g_wzPid;

// 四轮轮速和 IMU 加速度的恒加速度状态估计。
motion_compensation::OmniMotionCompensator g_motionCompensator;

// 四轮功率衰减器。输入为未限幅电流，输出为满足功率预算的电流。
power_control::PowerController g_powerController;

// 调试器可直接观察的功率控制快照。
float g_effectivePowerLimitW = kFallbackChassisPowerLimitW;
float g_estimatedPowerW = 0.0f;
float g_consumedPowerW = 0.0f;
power_control::PowerControlStatus g_powerStatus =
    power_control::PowerControlStatus::Disabled;

// 当前用于 PID 反馈和状态发布的底盘速度：通常是运动补偿融合值，
// 融合不可用时退回轮速一阶低通值。
float g_vxFiltered = 0.0f;
float g_vyFiltered = 0.0f;
float g_wzFiltered = 0.0f;

// 力控分配矩阵构建成功后才允许底盘输出。
bool g_forceReady = false;

// 默认使用自动模式，符合哨兵未选择操作方式时默认自动控制的规则。
ChassisModeTypedef g_chassisMode = CHASSIS_MODE_AUTO_VISION;

// 模式切换后的首周期先输出零，避免新控制源直接继承旧积分。
bool g_modeTransitionPending = false;

// 调试器可观察的当前控制源状态。
bool g_commandSourceReady = false;
uint32_t g_autoCommandLastUpdateMs = 0U;

// 运动补偿调试快照。
bool g_motionReady = false;
bool g_motionUsingAcceleration = false;
float g_motionAccelerationX = 0.0f;
float g_motionAccelerationY = 0.0f;

/**
 * @brief 将当前底盘速度快照发布到任务间共享状态。
 */
void PublishChassisVelocity(void) {
    LuojiaFox::system_state.updateChassis({
        g_vxFiltered,
        g_vyFiltered,
        g_wzFiltered
    });
}

/**
 * @brief 对归一化遥控器输入应用死区。
 * @param value 遥控器归一化输入。
 * @return 死区外保持原值，死区内返回 0。
 */
float ApplyDeadzone(float value) {
    return (basic_math::abs(value) < kRcDeadzone) ? 0.0f : value;
}

/**
 * @brief 清除三个速度环的积分项。
 * @note 底盘重新使能时调用，避免停车期间积分饱和造成起步冲击。
 */
void ResetVelocityPids(void) {
    g_vxPid.setIntegralError(0.0f);
    g_vyPid.setIntegralError(0.0f);
    g_wzPid.setIntegralError(0.0f);
}

/**
 * @brief 根据遥控拨杆选择底盘模式。
 *
 * 右拨杆下档具有最高优先级，用于立即锁定底盘。遥控在线时：
 * - 左拨杆上档：上位机/视觉自动模式；
 * - 左拨杆中档：直接遥控调试模式；
 * - 左拨杆下档：禁用。
 *
 * 遥控离线时保留最近一次模式，使自动哨兵可以不依赖遥控器运行；
 * 若保留的是直接遥控模式，后续会因遥控输入源不可用而停车。
 */
ChassisModeTypedef SelectChassisMode(bool rcOnline) {
    if (!rcOnline) {
        return g_chassisMode;
    }

    if (SWITCH_IS_DOWN(getDt7RcSw(RC_SW_RIGHT))) {
        return CHASSIS_MODE_DISABLED;
    }

    const char leftSwitch = getDt7RcSw(RC_SW_LEFT);
    if (SWITCH_IS_UP(leftSwitch)) {
        return CHASSIS_MODE_AUTO_VISION;
    }
    if (SWITCH_IS_MID(leftSwitch)) {
        return CHASSIS_MODE_DIRECT_RC;
    }
    return CHASSIS_MODE_DISABLED;
}

/**
 * @brief 读取当前模式对应的车体目标速度。
 * @param[out] vx 前后目标速度，m/s。
 * @param[out] vy 横移目标速度，m/s。
 * @param[out] wz 旋转目标速度，rad/s。
 * @return 当前控制源在线、数据新鲜且数值有效时返回 true。
 */
bool ReadTargetVelocity(
    const LuojiaFox::SystemStateSnapshot& state,
    float* vx,
    float* vy,
    float* wz
) {
    bool ready = false;

    if (g_chassisMode == CHASSIS_MODE_DIRECT_RC) {
        if (!getRcOffline()) {
            *vx = kRcXSign * ApplyDeadzone(
                getDt7RcCh(RC_CH_LEFT_VERTICAL)
            ) * kMaxVx;
            *vy = kRcYSign * ApplyDeadzone(
                getDt7RcCh(RC_CH_LEFT_HORIZONTAL)
            ) * kMaxVy;
            *wz = kRcWzSign * ApplyDeadzone(
                getDt7RcCh(RC_CH_RIGHT_HORIZONTAL)
            ) * kMaxWz;
            ready = true;
        }
    } else if (g_chassisMode == CHASSIS_MODE_AUTO_VISION) {
        const LuojiaFox::RobotCmdState& command = state.robot_cmd;
        g_autoCommandLastUpdateMs = command.last_update_ms;

        const bool fresh =
            command.last_update_ms != 0U
            && HAL_GetTick() - command.last_update_ms
                <= kAutoCommandTimeoutMs;
        const bool finite =
            std::isfinite(command.vx)
            && std::isfinite(command.vy)
            && std::isfinite(command.wz);

        if (fresh && finite) {
            *vx = basic_math::constrain(command.vx, -kMaxVx, kMaxVx);
            *vy = basic_math::constrain(command.vy, -kMaxVy, kMaxVy);
            *wz = basic_math::constrain(command.wz, -kMaxWz, kMaxWz);
            ready = true;
        }
    }

    g_commandSourceReady = ready;
    if (!ready) {
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
    }
    return ready;
}

/**
 * @brief 检查四个轮电机是否均在反馈超时时间内。
 * @return 四个轮电机全部在线时返回 true。
 */
bool AllWheelsOnline(void) {
    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        if (!g_wheelMotor[i].isOnline(now)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 获取当前有效的底盘功率上限。
 *
 * 裁判系统在 0x0201 中下发的 chassis_power_limit 已包含机器人类型、
 * 等级、姿态和已有功率增益，是比赛中的权威动态上限。超级电容模块在线时，
 * 还必须同时满足超级电容允许的功率上限。
 *
 * 裁判数据尚未建立时使用 60 W 台架回退值，避免上电阶段无限制输出。
 */
float ResolveChassisPowerLimit(void) {
    const RefereePowerSnapshot snapshot = referee.GetPowerSupplySnapshot();
    float limit = kFallbackChassisPowerLimitW;

    if (snapshot.online && snapshot.power_received) {
        // 收到有效 0x0201 后以裁判系统为准；0 W 表示当前禁止底盘输出。
        limit = static_cast<float>(snapshot.power_limit_w);
    }

    float supercap_limit = 0.0f;
    if (supercapGetChassisPowerLimit(&supercap_limit) && supercap_limit > 0.0f) {
        if (limit <= 0.0f || supercap_limit < limit) {
            limit = supercap_limit;
        }
    }

    return limit > 0.0f ? limit : 0.0f;
}

/**
 * @brief 按当前动态功率上限衰减四轮目标电流。
 * @param wheelOmega 四轮输出轴角速度，rad/s。
 * @param targetCurrent 力控计算得到的四轮目标电流，A。
 * @param outputCurrent 功率控制器衰减后的四轮电流，A。
 * @return 功率控制结果可信且满足预算时返回 true。
 *
 * 该函数每 1 ms 运行一次。裁判系统以 10 Hz 更新功率上限，
 * 因此这里每周期重新读取并应用最新上限。
 */
bool ApplyPowerControl(
    const float wheelOmega[kWheelCount],
    const float targetCurrent[kWheelCount],
    float outputCurrent[kWheelCount]
) {
    g_effectivePowerLimitW = ResolveChassisPowerLimit();
    power_control::powerControllerSetPowerLimit(
        &g_powerController,
        g_effectivePowerLimitW
    );

    power_control::MotorCommand motors[kWheelCount] = {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        motors[i].target_current = targetCurrent[i];
        motors[i].velocity = wheelOmega[i];
    }

    power_control::powerControllerUpdate(
        &g_powerController,
        motors,
        kWheelCount
    );
    g_powerStatus = g_powerController.status;
    g_estimatedPowerW = g_powerController.origin_power;
    g_consumedPowerW = g_powerController.consume_power;

    const bool valid_result =
        g_powerStatus == power_control::PowerControlStatus::Ready
        || g_powerStatus == power_control::PowerControlStatus::Limited;

    if (!valid_result) {
        for (uint8_t i = 0U; i < kWheelCount; ++i) {
            outputCurrent[i] = 0.0f;
        }
        return false;
    }

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        outputCurrent[i] = power_control::powerControllerLimitCurrent(
            &g_powerController,
            i
        );
    }
    return true;
}

/**
 * @brief 写入四个轮电机的控制量并发送一帧 0x200。
 * @param command 与逻辑轮序一致的 M3508 原始控制量，不是安培值。
 * @note DjiMotor::sendGroup() 会自动应用安装方向并填充对应 CAN 槽位。
 */
void SendCurrentCommand(const float command[kWheelCount]) {
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        float raw = command[i];
        // M3508/C620 的原始控制量范围为 -16384~16384。
        raw = basic_math::constrain(raw, -16384.0f, 16384.0f);
        g_wheelMotor[i].setCommand(static_cast<int16_t>(raw));
    }

    // 四个轮电机共享同一发送帧，因此只能在此处统一发送一次。
    (void)DjiMotor::sendGroup(&hfdcan1, kChassisTxId);
}

/**
 * @brief 关闭底盘输出，向四个轮电机发送零控制量。
 */
void SendZeroCurrent(void) {
    const float zero[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    SendCurrentCommand(zero);
}

/**
 * @brief 停止底盘输出并清除闭环历史状态。
 */
void StopChassis(void) {
    ResetVelocityPids();
    g_chassisForce.setEnable(false);
    SendZeroCurrent();
}

/**
 * @brief 由四个轮子的输出轴角速度反算车体速度。
 * @param wheelOmega 逻辑轮序对应的轮角速度，rad/s。
 * @param relativeYawRad 期望坐标系相对底盘坐标系的 Yaw 角，rad。
 * @param vx 输出前后速度，m/s。
 * @param vy 输出横移速度，m/s。
 * @param wz 输出旋转角速度，rad/s。
 * @return 四轮速度逆解和 wz 计算均成功时返回 true。
 */
bool EstimateBodyVelocity(
    const float wheelOmega[kWheelCount],
    uint8_t validWheelMask,
    float relativeYawRad,
    float *vx,
    float *vy,
    float *wz
) {
    float wheelLinear[kWheelCount];

    // 将角速度换算为轮缘线速度，供车体逆运动学使用。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        wheelLinear[i] = kWheelRadius * wheelOmega[i];
    }

    // vx/vy 复用运动补偿算法层的四轮逆解；当前仍要求四轮全部有效。
    if (!motion_compensation::OmniMotionCompensator::solveVelocity(
            wheelLinear,
            validWheelMask,
            relativeYawRad,
            vx,
            vy
        )) {
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
        return false;
    }

    // 纯旋转时每轮轮缘线速度为 -R*wz，因此四轮求和后使用轮半径反算角速度。
    *wz = -(
        wheelLinear[0] + wheelLinear[1] +
        wheelLinear[2] + wheelLinear[3]
    ) / (4.0f * kWheelRadius);

    return true;
}

}  // namespace

/**
 * @brief 初始化四个轮电机、力控分配器、速度环和功率控制器。
 * @note 必须在 bspFdcanInit() 之后、FreeRTOS 启动之前调用。
 *       任一电机配置或注册失败时进入 Error_Handler()。
 */
void Chassis_Init(void) {
    ForceActuatorParam params[kWheelCount] = {};

    // 1. 配置并注册四个 M3508；减速比和方向在此阶段一次性确定。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const uint32_t motorId = kWheelMotorId[i];

        if (!g_wheelMotor[i].init(
                MotorType::DjiM3508,
                static_cast<uint8_t>(motorId),
                &hfdcan1,
                kWheelDirection[i]
            ) ||
            !MotorBase::registerMotor(g_wheelMotor[i])) {
            Error_Handler();
        }
        g_wheelMotor[i].setOfflineTimeout(kOfflineTimeoutMs);

        // 力控参数使用轮输出轴坐标；电机安装方向已由 DjiMotor 处理。
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

    // 2. 初始化 4 执行器、3 自由度力控求解器。
    g_chassisForce.init(params, kWheelCount, 3U, 0.001f);
    g_chassisForce.setVelocityFilterAlpha(0.8f);

    // 初始化轮速 + IMU 恒加速度运动估计，默认参数需通过实车日志继续标定。
    g_motionReady = g_motionCompensator.init();
    g_motionUsingAcceleration = false;
    g_motionAccelerationX = 0.0f;
    g_motionAccelerationY = 0.0f;

    // 构建四轮全向底盘分配矩阵；失败时 g_forceReady 保持 false。
    float distribution[kWheelCount * 3U] = {};

    g_forceReady = ForceSolver::buildOmniDistribution(
        kWheelRadius,
        kWheelCenterDistance,
        distribution
    );

    if (g_forceReady) {
        g_chassisForce.setDistribution(distribution);
    }

    // 3. 初始化 vx、vy、wz 三个速度环。
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

    // 4. 初始化功率模型。最终动态上限在控制周期中读取裁判系统后设置，
    //    这里先使用保守回退值；裁判系统 0x0201 已包含姿态和增益效果，
    //    因此禁用算法层内部的二次姿态缩放。
    power_control::powerControllerInit(
        &g_powerController,
        &power_control::kDefaultC620Coefficients,
        kFallbackChassisPowerLimitW
    );
    g_powerController.margin = kPowerSafetyMargin;
    g_powerController.posture_scale = {1.0f, 1.0f, 1.0f, 1.0f};
    power_control::powerControllerSetCurrentLimit(
        &g_powerController,
        kC620MaxCurrent
    );
    g_effectivePowerLimitW = kFallbackChassisPowerLimitW;
    g_estimatedPowerW = 0.0f;
    g_consumedPowerW = 0.0f;
    g_powerStatus = power_control::PowerControlStatus::Disabled;

    // 5. 清空反馈状态，并确保初始化结束时电机输出为零。
    g_vxFiltered = 0.0f;
    g_vyFiltered = 0.0f;
    g_wzFiltered = 0.0f;

    SendZeroCurrent();
}

/**
 * @brief 执行一次底盘控制周期。
 * @note 正常周期约为 1 ms：读取反馈 -> 估计车体速度 -> 安全检查
 *       -> 遥控速度闭环 -> 力控分配 -> CAN 电流命令。
 */
void Chassis_Control(void) {
    float wheelOmega[kWheelCount];
    float wheelLinear[kWheelCount];
    const LuojiaFox::SystemStateSnapshot state =
        LuojiaFox::system_state.read();
    const uint32_t now = HAL_GetTick();
    uint8_t validWheelMask = 0U;

    // 1. 读取新电机驱动换算后的轮输出轴角速度。
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        const DjiMotorData feedback = g_wheelMotor[i].data();
        wheelOmega[i] = feedback.velocity_rad_s;
        wheelLinear[i] = kWheelRadius * wheelOmega[i];
        if (g_wheelMotor[i].isOnline(now) && std::isfinite(wheelOmega[i])) {
            validWheelMask |= static_cast<uint8_t>(1U << i);
        }
    }

    // 2. 先做轮速逆解，再使用轮速和 IMU 加速度进行恒加速度状态融合。
    //    云台与底盘相对 Yaw 尚未接入，当前使用 0 rad 保持底盘坐标。
    float vxRaw = 0.0f;
    float vyRaw = 0.0f;
    float wzRaw = 0.0f;
    constexpr float kRelativeYawRad = 0.0f;
    const bool velocityEstimateValid = EstimateBodyVelocity(
        wheelOmega,
        validWheelMask,
        kRelativeYawRad,
        &vxRaw,
        &vyRaw,
        &wzRaw
    );

    const bool imuFresh =
        state.imu.last_update_ms != 0U
        && now - state.imu.last_update_ms <= kImuDataTimeoutMs;

    motion_compensation::Measurement measurement {};
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        measurement.wheel_linear_velocity_mps[i] = wheelLinear[i];
    }
    measurement.valid_wheel_mask = validWheelMask;
    measurement.yaw_rad = kRelativeYawRad;
    measurement.yaw_rate_rad_s =
        imuFresh ? state.imu.yaw_rate_dps * BASIC_MATH_DEG_TO_RAD : 0.0f;
    measurement.acceleration_x_mps2 =
        imuFresh ? state.imu.acceleration_x_mps2 : 0.0f;
    measurement.acceleration_y_mps2 =
        imuFresh ? state.imu.acceleration_y_mps2 : 0.0f;
    measurement.acceleration_valid =
        imuFresh && state.imu.acceleration_valid;
    measurement.dt_s = kMotionControlPeriodS;

    const bool motionUpdated =
        g_motionReady && g_motionCompensator.update(measurement);
    if (motionUpdated) {
        const motion_compensation::Estimate motionEstimate =
            g_motionCompensator.estimate();
        g_vxFiltered = motionEstimate.vx_mps;
        g_vyFiltered = motionEstimate.vy_mps;
        g_motionUsingAcceleration = measurement.acceleration_valid;
        g_motionAccelerationX = motionEstimate.ax_mps2;
        g_motionAccelerationY = motionEstimate.ay_mps2;
    } else {
        // IMU 数据不可用时退回原有轮速低通，不改变底盘启停安全策略。
        g_vxFiltered = kVelocityFilterAlpha * g_vxFiltered +
                       (1.0f - kVelocityFilterAlpha) * vxRaw;
        g_vyFiltered = kVelocityFilterAlpha * g_vyFiltered +
                       (1.0f - kVelocityFilterAlpha) * vyRaw;
        g_motionUsingAcceleration = false;
        g_motionAccelerationX = 0.0f;
        g_motionAccelerationY = 0.0f;
    }

    g_wzFiltered = kVelocityFilterAlpha * g_wzFiltered +
                   (1.0f - kVelocityFilterAlpha) * wzRaw;

    PublishChassisVelocity();

    // 3. 选择控制模式。模式切换的首个周期先停车并清除历史状态。
    const bool rcOnline = !getRcOffline();
    const ChassisModeTypedef requestedMode = SelectChassisMode(rcOnline);
    if (requestedMode != g_chassisMode) {
        g_chassisMode = requestedMode;
        g_modeTransitionPending = true;
    }

    if (g_modeTransitionPending) {
        g_modeTransitionPending = false;
        g_commandSourceReady = false;
        StopChassis();
        return;
    }

    // 4. 读取当前模式对应的控制源。各模式只检查自己的输入是否有效。
    float targetVx = 0.0f;
    float targetVy = 0.0f;
    float targetWz = 0.0f;
    const bool commandReady = ReadTargetVelocity(
        state,
        &targetVx,
        &targetVy,
        &targetWz
    );

    if ((!velocityEstimateValid) || (!commandReady) ||
        (!AllWheelsOnline()) || (!g_forceReady)) {
        g_vxFiltered = vxRaw;
        g_vyFiltered = vyRaw;
        g_wzFiltered = wzRaw;
        PublishChassisVelocity();

        StopChassis();
        return;
    }

    // 5. 根据旋转角速度对目标平移速度施加受限补偿。
    if (kEnableMotionTargetCompensation
        && g_motionCompensator.isInitialized()) {
        const motion_compensation::CorrectedVelocity corrected =
            g_motionCompensator.compensate(targetVx, targetVy);
        targetVx = basic_math::constrain(corrected.vx_mps, -kMaxVx, kMaxVx);
        targetVy = basic_math::constrain(corrected.vy_mps, -kMaxVy, kMaxVy);
    }

    // 6. 将当前控制源的目标速度送入三个速度环。
    g_vxPid.setTarget(targetVx);
    g_vyPid.setTarget(targetVy);
    g_wzPid.setTarget(targetWz);

    // 更新速度环反馈并计算三个方向的广义力。
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

    // 6. 使用当前轮速和广义力计算每个轮子的输出轴力矩。
    g_chassisForce.setTarget(generalizedForce);
    g_chassisForce.setNow(nullptr, wheelOmega);
    g_chassisForce.setEnable(true);
    g_chassisForce.timCalculatePeriodElapsedCallback();

    // 7. 先按规则功率预算衰减目标电流，再换算成 C620 原始控制量。
    float targetCurrent[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    float limitedCurrent[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        targetCurrent[i] = basic_math::constrain(
            g_chassisForce.getOut(i),
            -kC620MaxCurrent,
            kC620MaxCurrent
        );
    }

    if (!ApplyPowerControl(wheelOmega, targetCurrent, limitedCurrent)) {
        ResetVelocityPids();
        g_chassisForce.setEnable(false);
        SendZeroCurrent();
        return;
    }

    constexpr float currentToCommand =
        16384.0f / kC620MaxCurrent;
    float currentCommand[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        currentCommand[i] =
            limitedCurrent[i] * currentToCommand;
    }

    // 8. 更新四个电机的控制量并发送共享 CAN 帧。
    SendCurrentCommand(currentCommand);
}

ChassisModeTypedef Chassis_GetMode(void) {
    return g_chassisMode;
}

#if BOARD_ROLE == 1
/**
 * @brief 底盘控制 FreeRTOS 任务入口。
 * @param argument 未使用。
 * @note 仅在底盘板编译并执行，任务以约 1 ms 周期调用 Chassis_Control()。
 */
extern "C" void Control_Task(void const *argument) {
    (void)argument;

    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
#endif
