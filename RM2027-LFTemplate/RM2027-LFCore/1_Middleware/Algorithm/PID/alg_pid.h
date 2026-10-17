/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef ALG_PID_H
    #define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.hpp"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行枚举
 *
 */
enum PidDFirst_e {
    PidDFirstDisable = 0,
    PidDFirstEnable,
};

/**
 * @brief PID控制器类
 *        继承自 Controller 基类
 *        使用小驼峰命名规范
 *
 */
class PID  {
public:
    /**
     * @brief 初始化控制器（重写基类纯虚函数）
     *        使用默认参数初始化PID控制器
     *
     */
    void init();

    /**
     * @brief PID参数配置（PID特有方法）
     *        配置PID控制器的所有参数
     *
     * @param kp P值
     * @param ki I值
     * @param kd D值
     * @param kf 前馈 (默认: 0.0f)
     * @param iOutMax 积分限幅 (默认: 0.0f)
     * @param outMax 输出限幅 (默认: 0.0f)
     * @param dt 时间片长度 (默认: 0.001f)
     * @param deadZone 死区误差阈值 (默认: 0.0f)
     * @param iVariableSpeedA 变速积分误差阈值A (默认: 0.0f)
     * @param iVariableSpeedB 变速积分误差阈值B (默认: 0.0f)
     * @param iSeparateThreshold 积分分离误差阈值 (默认: 0.0f)
     * @param dFirst 是否开启微分先行 (默认: PID_D_First_DISABLE)
     */
    void init(
        float kp,
        float ki,
        float kd,
        float kf = 0.0f,
        float iOutMax = 0.0f,
        float outMax = 0.0f,
        float dt = 0.001f,
        float deadZone = 0.0f,
        float iVariableSpeedA = 0.0f,
        float iVariableSpeedB = 0.0f,
        float iSeparateThreshold = 0.0f,
        PidDFirst_e dFirst = PidDFirstDisable
    );

    inline float getIntegralError();

    inline void setKP(float kp);

    inline void setKI(float ki);

    inline void setKD(float kd);

    inline void setKF(float kf);

    inline void setIOutMax(float iOutMax);

    inline void setOutMax(float outMax);

    inline void setIVariableSpeedA(float iVariableSpeedA);

    inline void setIVariableSpeedB(float iVariableSpeedB);

    inline void setISeparateThreshold(float iSeparateThreshold);

    inline void setIntegralError(float integralError);

    /**
     * @brief 计算PID输出（重写基类的定时器回调方法）
     *        在定时器中断中周期性调用
     *
     */
    void timCalculatePeriodElapsedCallback();

    /**
     * @brief 获取输出值（重写基类方法）
     *
     * @return float PID输出值
     */
    float getOut();

    /**
     * @brief 设置目标值（重写基类方法）
     *
     * @param target 目标值
     */
    void setTarget(float target);

    /**
     * @brief 设置当前值/反馈值（重写基类方法）
     *
     * @param now 当前值
     */
    void setNow(float now);



protected:
    // 初始化相关常量

    // PID计时器周期, s
    float Dt;
    // 死区, Error在其绝对值内不输出
    float DeadZone;
    // 微分先行
    PidDFirst_e DFirst;

    // 常量

    // 内部变量

    // 之前的当前值
    float PreNow = 0.0f;
    // 之前的目标值
    float PreTarget = 0.0f;
    // 之前的输出值
    float PreOut = 0.0f;
    // 前向误差
    float PreError = 0.0f;

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // PID的P
    float KP = 0.0f;
    // PID的I
    float KI = 0.0f;
    // PID的D
    float KD = 0.0f;
    // 前馈
    float KF = 0.0f;

    // 积分限幅, 0为不限制
    float IOutMax = 0;
    // 输出限幅, 0为不限制
    float OutMax = 0;

    // 变速积分定速内段阈值, 0为不限制
    float IVariableSpeedA = 0.0f;
    // 变速积分变速区间, 0为不限制
    float IVariableSpeedB = 0.0f;
    // 积分分离阈值，需为正数, 0为不限制
    float ISeparateThreshold = 0.0f;

    // 目标值
    float Target = 0.0f;
    // 当前值
    float Now = 0.0f;

    // 读写变量

    // 积分值
    float IntegralError = 0.0f;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取积分误差值
 *
 * @return float 积分误差值
 */
inline float PID::getIntegralError() {
    return (IntegralError);
}

/**
 * @brief 获取输出值（重写基类方法）
 *
 * @return float 输出值
 */
inline float PID::getOut() {
    return (Out);
}

/**
 * @brief 设定PID的P
 *
 * @param kp PID的P
 */
inline void PID::setKP(float kp) {
    KP = kp;
}

/**
 * @brief 设定PID的I
 *
 * @param ki PID的I
 */
inline void PID::setKI(float ki) {
    KI = ki;
}

/**
 * @brief 设定PID的D
 *
 * @param kd PID的D
 */
inline void PID::setKD(float kd) {
    KD = kd;
}

/**
 * @brief 设定前馈
 *
 * @param kf 前馈
 */
inline void PID::setKF(float kf) {
    KF = kf;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param iOutMax 积分限幅, 0为不限制
 */
inline void PID::setIOutMax(float iOutMax) {
    IOutMax = iOutMax;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param outMax 输出限幅, 0为不限制
 */
inline void PID::setOutMax(float outMax) {
    OutMax = outMax;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param iVariableSpeedA 定速内段阈值, 0为不限制
 */
inline void PID::setIVariableSpeedA(float iVariableSpeedA) {
    IVariableSpeedA = iVariableSpeedA;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param iVariableSpeedB 变速区间, 0为不限制
 */
inline void PID::setIVariableSpeedB(float iVariableSpeedB) {
    IVariableSpeedB = iVariableSpeedB;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param iSeparateThreshold 积分分离阈值，需为正数, 0为不限制
 */
inline void PID::setISeparateThreshold(float iSeparateThreshold) {
    ISeparateThreshold = iSeparateThreshold;
}

/**
 * @brief 设定目标值（重写基类方法）
 *
 * @param target 目标值
 */
inline void PID::setTarget(float target) {
    Target = target;
}

/**
 * @brief 设定当前值/反馈值（重写基类方法）
 *
 * @param now 当前值
 */
inline void PID::setNow(float now) {
    Now = now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param integralError 积分值
 */
inline void PID::setIntegralError(float integralError) {
    IntegralError = integralError;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/