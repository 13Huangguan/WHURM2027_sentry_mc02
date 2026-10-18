/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       alg_power_control.hpp
  * @brief      底盘功率控制（功率重分配和衰减电流法）（解算方法参考中科大）
  * @note       只对行进轮做功率控制，转向轮不参与。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-18      XeuLes          1. 参考中科大两种功率控制，尚未测电机六参数
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#ifndef ALG_POWER_CONTROL_HPP
#define ALG_POWER_CONTROL_HPP

#include <stdint.h>

#ifndef POWER_CONTROL_USE_POWER_DECAY
#define POWER_CONTROL_USE_POWER_DECAY 0
#endif

#define POWER_CONTROL_MAX_MOTOR 8

namespace power_control {

//电机功率模型
struct ModelCoefficients {
    float k0; /*!< 常数项（电调自身损耗） */
    float k1; /*!< I 项系数（电调电流相关损耗） */
    float k2; /*!< w 项系数（粘滞摩擦） */
    float k3; /*!< I*w 项系数 */
    float k4; /*!< I*I 项系数 */
    float k5; /*!< w*w 项系数 */
};

struct MotorCommand {
    float target_current; /*!< 解算出来的原始目标电流，A */
    float velocity;       /*!< 当前角速度，rad/s */
};

//哨兵姿态
enum class SentryPosture : uint8_t {
    Unknown = 0,
    Attack  = 1,
    Defence = 2,
    Move    = 3,
};

//各姿态的功率倍率
struct PostureScale {
    float attack;
    float defence;
    float move;
    float unknown;
};

//功率控制器状态
struct PowerController {
    ModelCoefficients coeff; /*!< 行进电机模型系数 */
    PostureScale posture_scale;

    float estimate[POWER_CONTROL_MAX_MOTOR]; /*!< 各轮衰前功率估计值，W */
    float output[POWER_CONTROL_MAX_MOTOR];   /*!< 各轮衰减后的输出电流，A */
    uint8_t count;                           /*!< 本轮电机数量 */

    float referee_power_limit; /*!< 裁判系统给的底盘功率上限，W */
    float margin;              /*!< 安全裕度，本轮预算 = 上限 * margin */
    float available_power;     /*!< 本轮预算（已乘姿态倍率和裕度），W */
    float origin_power;        /*!< 衰减前总功率，W（含发电回流，可为负） */
    float consume_power;       /*!< 衰减后预计总功率，W（含发电回流） */
    float factor;              /*!< 电流法为 η，功率法为 ρ；1.0 表示不衰减 */
    float current_limit;       /*!< 输出电流的绝对值限幅，A */
    SentryPosture posture;     /*!< 当前姿态 */
};

/* C620 电调 + M3508 电机的默认模型系数 */
extern const ModelCoefficients kDefaultC620Coefficients;

/* 默认姿态倍率 */
extern const PostureScale kDefaultPostureScale;

/**
  * @brief          功率控制器初始化
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      coeff: 行进电机模型系数，传 nullptr 用 kDefaultC620Coefficients
  * @param[in]      referee_power_limit: 裁判系统底盘功率上限，W
  * @retval         none
  */
void powerControllerInit(
    PowerController *ctrl,
    const ModelCoefficients *coeff,
    float referee_power_limit
);

/**
  * @brief          设置行进电机模型系数
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      coeff: 电机模型系数
  * @retval         none
  */
void powerControllerSetCoefficients(PowerController *ctrl, const ModelCoefficients *coeff);

/**
  * @brief          设置裁判系统给的底盘功率上限
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      limit: 功率上限，W
  * @retval         none
  * @note           每个控制周期都应该更新，裁判系统会随等级/增益变化
  */
void powerControllerSetRefereeLimit(PowerController *ctrl, float limit);

/**
  * @brief          设置哨兵姿态
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      posture: 姿态
  * @retval         none
  */
void powerControllerSetPosture(PowerController *ctrl, SentryPosture posture);

/**
  * @brief          设置输出电流的绝对值限幅
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      limit: 电流绝对值上限，A
  * @retval         none
  */
void powerControllerSetCurrentLimit(PowerController *ctrl, float limit);

/**
  * @brief          单个电机的功率估计
  * @param[in]      coeff: 电机模型系数
  * @param[in]      current: 电流，A
  * @param[in]      velocity: 角速度，rad/s
  * @retval         估计功率，W
  */
float powerEstimate(const ModelCoefficients *coeff, float current, float velocity);

/**
  * @brief          更新本轮功率状态，一次算好所有电机的输出电流
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      motors: 本轮所有行进电机的指令电流与角速度
  * @param[in]      count: 电机数量，超过 POWER_CONTROL_MAX_MOTOR 会被截断
  */
void powerControllerUpdate(PowerController *ctrl, const MotorCommand *motors, uint8_t count);

/**
  * @brief          取第 i 个行进电机衰减后的输出电流
  * @param[in]      ctrl: 控制器结构指针（需先调用 powerControllerUpdate）
  * @param[in]      index: 电机序号，对应 Update 时 motors[] 的下标
  * @retval         衰减并限幅后的电流，A；索引越界返回 0
  */
float powerControllerLimitCurrent(const PowerController *ctrl, uint8_t index);

/**
  * @brief          取本轮预算
  * @param[in]      ctrl: 控制器结构指针
  * @retval         预算，W
  */
float powerControllerAvailablePower(const PowerController *ctrl);

/**
  * @brief          取衰减前总功率
  * @param[in]      ctrl: 控制器结构指针
  * @retval         总功率，W（含发电回流，可为负）
  */
float powerControllerOriginPower(const PowerController *ctrl);

/**
  * @brief          取衰减后预计总功率
  * @param[in]      ctrl: 控制器结构指针
  * @retval         总功率，W（含发电回流，可为负）
  */
float powerControllerConsumePower(const PowerController *ctrl);

/**
  * @brief          取当前衰减系数
  * @param[in]      ctrl: 控制器结构指针
  * @retval         电流法为 η，功率法为 ρ；1.0 表示无需衰减
  */
float powerControllerFactor(const PowerController *ctrl);

} // namespace power_control

#endif // ALG_POWER_CONTROL_HPP

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
