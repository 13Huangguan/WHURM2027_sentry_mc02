/**
 * @file alg_force.h
 * @author WHU-LuojiaFox
 * @brief 力控解算：期望广义力 -> 各执行器力矩 -> 电机侧控制量
 * @version 1.0
 * @date 2026-10-13 1.0 首版
 *
 * @copyright WHU-LuojiaFox (c) 2026
 *
 * 只依赖 C 标准库，不含 HAL / FreeRTOS / CMSIS-DSP，
 * 因此同一份文件既能编进固件，也能在 PC 上用 g++ 直接跑自检
 * （编译时定义 ALG_FORCE_HOST_TEST，见 alg_force.cpp 末尾）。
 *
 * 解算链路（都在 timCalculatePeriodElapsedCallback 里）：
 *   1. 分配       tau_i = sum_j M[i][j] * F[j]              期望广义力 -> 执行器力矩
 *   2. 动力学补偿 tau_i += J*qdd + b*qd + tau_c*sgn(qd)      惯量 / 粘滞摩擦 / 库仑摩擦
 *   3. 重力矩补偿 tau_i += A*cos(theta - phi)                随位姿变化的静重力矩
 *   4. 前馈与限幅 tau_i  = clamp(tau_i + ff_i, -tauMax, tauMax)
 *   5. 执行器换算 i     = tau / (Kt * ratio) * dir           力矩 -> 电机侧控制量
 *
 * 典型调用（每个控制周期一次）：
 *   solver.setEnable(!offline);
 *   solver.setTarget(generalizedForce);   // 期望广义力
 *   solver.setNow(position, velocity);    // 本轮反馈，不需要的项传 nullptr
 *   solver.timCalculatePeriodElapsedCallback();
 *   float torque = solver.getTorque(0);   // N*m，给达妙 MIT 的 Torque 字段
 *   float amps   = solver.getOut(0);      // A，给 DJI 电流环
 */

#ifndef ALG_FORCE_H
#define ALG_FORCE_H

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/*!< 单个解算器支持的最大执行器数量 */
#define ALG_FORCE_ACTUATOR_MAX 8
/*!< 单个解算器支持的广义力维数 */
#define ALG_FORCE_DOF_MAX 4

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单个执行器的模型参数
 *        除 Direction 外，其余物理量均折算到执行器输出轴
 */
struct ForceActuatorParam_t {
    float TorqueConstant;   /*!< (N*m/A)     电机力矩常数 Kt */
    float ReductionRatio;   /*!< (none)      减速比，输出轴/电机轴 */
    int8_t Direction;       /*!< (none)      输出轴正方向与机构正方向的关系，取 1 或 -1 */
    float TorqueMax;        /*!< (N*m)       输出轴力矩限幅 */
    float Inertia;          /*!< (kg*m^2)    折算到输出轴的等效转动惯量 */
    float ViscousFriction;  /*!< (N*m*s/rad) 粘滞摩擦系数 b */
    float CoulombFriction;  /*!< (N*m)       库仑摩擦幅值 tau_c */
    float GravityAmplitude; /*!< (N*m)       重力矩幅值 A，填 0 表示该轴不做重力补偿 */
    float GravityOffset;    /*!< (rad)       重力矩相位 phi，重力矩取最大时的角度 */
};

/**
 * @brief 解算器使能状态
 */
enum ForceSolverStatus_e {
    ForceSolverStatus_Disable = 0, /*!< 失能，输出恒为 0 */
    ForceSolverStatus_Enable,      /*!< 使能 */
};

/**
 * @brief 力控解算器
 */
class ForceSolver {
public:
    /**
     * @brief 初始化解算器
     *
     * @param param        执行器参数数组，长度不小于 actuatorNum
     * @param actuatorNum  执行器数量，上限 ALG_FORCE_ACTUATOR_MAX
     * @param dofNum       广义力维数，单关节填 1，底盘填 3
     * @param dt           解算周期, s
     */
    void init(const ForceActuatorParam_t *param, uint8_t actuatorNum, uint8_t dofNum, float dt);

    /**
     * @brief 设置分配矩阵（广义力 -> 执行器力矩）
     *
     * @param matrix 行优先矩阵首地址，尺寸 actuatorNum x dofNum
     * @note 不设置时默认为单位阵，即第 i 维广义力只驱动第 i 个执行器
     */
    void setDistribution(const float *matrix);

    /**
     * @brief 设置期望广义力
     *
     * @param generalizedForce 期望力数组，长度 dofNum
     */
    void setTarget(const float *generalizedForce);

    /**
     * @brief 设置本轮反馈
     *
     * @param position 位置数组，长度 actuatorNum
     * @param velocity 速度数组，长度 actuatorNum
     * @note 传 nullptr 表示本周期不更新该项
     */
    void setNow(const float *position, const float *velocity);

    /**
     * @brief 设置额外前馈力矩，用于补偿参数表里没有的项
     *
     * @param torque 前馈力矩数组，长度 actuatorNum
     */
    void setFeedforwardTorque(const float *torque);

    /**
     * @brief 设置速度反馈的低通滤波系数
     *
     * @param alpha 滤波系数 [0, 1]，越大越平滑
     */
    void setVelocityFilterAlpha(float alpha);

    /**
     * @brief 使能/失能，失能时输出与内部状态清零
     *
     * @param enable true 使能
     */
    void setEnable(bool enable);

    /**
     * @brief 周期解算，以 1 / dt 的频率调用
     */
    void timCalculatePeriodElapsedCallback();

    /**
     * @brief 清零全部状态量，切模式或异常后调用
     */
    void reset();

    /**
     * @brief 生成四轮全向底盘的分配矩阵（最小范数解）
     *
     * 由虚功原理，广义力 F=(Fx, Fy, Mz) 与轮牵引力 f 的关系是 B^T f = F，
     * 三方程四未知数，取最小范数解 f = B (B^T B)^-1 F。其中
     *     B^T B = diag(2, 2, 4*L^2)
     * 于是每个轮子的输出轴力矩为
     *     tau_i = r*(c_i/2)*Fx + r*(d_i/2)*Fy - (r/(4L))*Mz
     * c = (+0.7071, +0.7071, -0.7071, -0.7071)
     * d = (-0.7071, +0.7071, +0.7071, -0.7071)
     * 轮子编号与 chassis_omni 里 ChassisConsole 的四条式子一致。
     *
     * @param wheelRadius          轮子半径 r, m
     * @param wheelCenterDistance  轮心到车体中心距离 L, m
     * @param matrix               输出缓冲，行优先 4 x 3，至少 12 个 float
     * @return true 生成成功；matrix 为空或 L 为 0 时返回 false
     */
    static bool buildOmniDistribution(
        float wheelRadius,
        float wheelCenterDistance,
        float *matrix
    );

    inline bool isEnable() const;

    inline uint8_t getActuatorNum() const;

    inline uint8_t getDofNum() const;

    /**
     * @brief 获取执行器输出轴力矩
     *
     * @param index 执行器下标
     * @return float 力矩, N*m
     */
    inline float getTorque(uint8_t index) const;

    /**
     * @brief 获取电机侧控制量
     *
     * @param index 执行器下标
     * @return float 控制量，电流环为 A
     */
    inline float getOut(uint8_t index) const;

    /**
     * @brief 获取滤波后的反馈速度，便于调试观察
     *
     * @param index 执行器下标
     * @return float 速度，单位与输入一致
     */
    inline float getVelocity(uint8_t index) const;

    /**
     * @brief 获取本周期使用的期望广义力
     *
     * @param index 广义力下标
     * @return float 期望力
     */
    inline float getGeneralizedForce(uint8_t index) const;

protected:
    // 初始化相关常量

    // 执行器数量
    uint8_t ActuatorNum = 0;
    // 广义力维数
    uint8_t DofNum = 0;
    // 解算周期, s
    float Dt = 0.001f;
    // 速度反馈低通滤波系数
    float VelocityFilterAlpha = 0.9f;

    // 执行器参数
    ForceActuatorParam_t Param[ALG_FORCE_ACTUATOR_MAX] = {};

    // 分配矩阵
    float Distribution[ALG_FORCE_ACTUATOR_MAX][ALG_FORCE_DOF_MAX] = {};

    // 目标值

    // 期望广义力
    float TargetGeneralizedForce[ALG_FORCE_DOF_MAX] = {};
    // 额外前馈力矩
    float FeedforwardTorque[ALG_FORCE_ACTUATOR_MAX] = {};

    // 反馈值

    // 位置
    float Position[ALG_FORCE_ACTUATOR_MAX] = {};
    // 速度，已滤波
    float Velocity[ALG_FORCE_ACTUATOR_MAX] = {};
    // 上一周期速度，用于差分求加速度
    float PreVelocity[ALG_FORCE_ACTUATOR_MAX] = {};
    // 速度低通滤波器状态
    float VelocityFilterOutput[ALG_FORCE_ACTUATOR_MAX] = {};
    // 速度低通滤波器是否已用当前输入初始化
    bool VelocityFilterReady[ALG_FORCE_ACTUATOR_MAX] = {};

    // 输出值

    // 执行器输出轴力矩
    float Torque[ALG_FORCE_ACTUATOR_MAX] = {};
    // 电机侧控制量
    float Out[ALG_FORCE_ACTUATOR_MAX] = {};

    // 状态
    ForceSolverStatus_e Status = ForceSolverStatus_Disable;

private:
    /**
     * @brief 限幅
     *
     * @param value    输入
     * @param minValue 下限
     * @param maxValue 上限
     * @return float 限幅后的值
     */
    static float constrainValue(float value, float minValue, float maxValue);
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取使能状态
 *
 * @return true 已使能
 */
inline bool ForceSolver::isEnable() const {
    return (Status == ForceSolverStatus_Enable);
}

/**
 * @brief 获取执行器数量
 *
 * @return uint8_t 执行器数量
 */
inline uint8_t ForceSolver::getActuatorNum() const {
    return (ActuatorNum);
}

/**
 * @brief 获取广义力维数
 *
 * @return uint8_t 维数
 */
inline uint8_t ForceSolver::getDofNum() const {
    return (DofNum);
}

inline float ForceSolver::getTorque(uint8_t index) const {
    if (index >= ActuatorNum) {
        return (0.0f);
    }
    return (Torque[index]);
}

inline float ForceSolver::getOut(uint8_t index) const {
    if (index >= ActuatorNum) {
        return (0.0f);
    }
    return (Out[index]);
}

inline float ForceSolver::getVelocity(uint8_t index) const {
    if (index >= ActuatorNum) {
        return (0.0f);
    }
    return (Velocity[index]);
}

inline float ForceSolver::getGeneralizedForce(uint8_t index) const {
    if (index >= DofNum) {
        return (0.0f);
    }
    return (TargetGeneralizedForce[index]);
}

#endif

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/