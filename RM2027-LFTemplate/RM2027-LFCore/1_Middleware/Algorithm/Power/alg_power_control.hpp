/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       alg_power_control.hpp
  * @brief      底盘功率控制（功率重分配和衰减电流法）（解算方法参考中科大）
  * @note       只对行进轮做功率控制，转向轮不参与。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-18      XeuLes          1. 两种功率控制
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

// 四参数模型：P = k0 + k3*I*w + k4*I^2 + k5*w^2。
struct ModelCoefficients {
    float k0; /*!< 常数项（电调自身损耗） */
    float k3; /*!< I*w 项系数 */
    float k4; /*!< I*I 项系数 */
    float k5; /*!< w*w 项系数 */
};

enum class PowerControlStatus : uint8_t {
    Ready = 0,       /*!< 无需衰减 */
    Limited,         /*!< 已衰减且最终模型功率不超过预算 */
    Disabled,        /*!< 硬上限/策略预算为零，输出零电流 */
    BudgetInfeasible, /*!< 允许电流范围内无法达到预算；输出最小模型功率点 */
    InvalidInput,    /*!< 非有限输入、非法系数或运算溢出；输出零电流 */
};

struct MotorCommand {
    float target_current; /*!< 调用方已完成电流限幅的目标，A */
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

    float estimate[POWER_CONTROL_MAX_MOTOR]; /*!< 输入电流对应的各轮模型功率，W */
    float output[POWER_CONTROL_MAX_MOTOR];   /*!< 各轮衰减后的输出电流，A */
    uint8_t count;                           /*!< 本轮电机数量 */

    float referee_power_limit; /*!< 兼容旧字段名：现表示超电返回的允许功率硬上限，W */
    float margin;              /*!< 安全裕度，本轮预算 = 上限 * margin */
    float available_power;     /*!< 本轮预算（已乘姿态倍率和裕度），W */
    float origin_power;        /*!< 衰减前总功率，W（含发电回流，可为负） */
    float consume_power;       /*!< 衰减后预计总功率，W（含发电回流） */
    float factor;              /*!< 电流法为 η；功率法为可调功率的保留比例 ρ */
    float current_limit;       /*!< 输入约定的电流上界，A；只校验，不重复限幅 */
    SentryPosture posture;     /*!< 当前姿态 */
    float minimum_power;      /*!< 当前方法允许电流范围内的最低模型总功率，W */
    bool budget_feasible;     /*!< 最终输出模型功率确实 <= available_power */
    PowerControlStatus status;
};

/* C620 电调 + M3508 电机的默认模型系数 */
extern const ModelCoefficients kDefaultC620Coefficients;

/* 默认姿态倍率 */
extern const PostureScale kDefaultPostureScale;

/**
  * @brief          功率控制器初始化
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      coeff: 行进电机模型系数，传 nullptr 用 kDefaultC620Coefficients
  * @param[in]      referee_power_limit: 超电允许功率硬上限，W；0 保持零电流，无 45W 兜底
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
  * @brief          设置超电返回的功率硬上限；0 明确关闭输出
  */
void powerControllerSetPowerLimit(PowerController *ctrl, float limit);

/**
  * @brief          兼容旧名称，等同 powerControllerSetPowerLimit()
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      limit: 功率上限，W
  * @retval         none
  * @note           每周期更新；超电预算不可用时传 0，不保留陈旧上限
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
  * @brief          设置调用方已采用的电流上界，用于校验输入范围
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
  * @param[in]      count: 电机数量；超出容量或输入异常时状态为 InvalidInput
  * @note           调用前完成电流限幅；超出约定范围时返回 InvalidInput，不静默截断。
  *                 输出单位 A；速度轴和模型标定一致。需读取 status/budget_feasible，
  *                 预算不可达时不能把最小功率输出理解为已满足上限。
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
