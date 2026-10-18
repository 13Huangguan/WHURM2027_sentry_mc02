/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       alg_power_control.cpp
  * @brief      底盘功率控制（只控制行进轮）
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-18      XeuLes          1. 参考中科大两种功率控制，尚未测电机六参数
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#include "alg_power_control.hpp"

#include <math.h>
#include <string.h>

namespace power_control {

/* ------------------------------------------------------------------------- */
/* 默认参数                                                                   */
/* ------------------------------------------------------------------------- */

/**
 * @brief C620 电调 + M3508 电机的功率模型系数
 * @note  来自 2026 赛季整车实测标定。
 *        电流单位为 A，角速度单位为 rad/s。
 *        这组值是按旧 4 系数模型标定的，k1、k2 两项没有数据，置 0；
 *        重新做 6 系数辨识后可以把这两项补上。
 */
const ModelCoefficients kDefaultC620Coefficients = {
    1.598658371452403f,      /* k0  常数项（原 a） */
    0.0f,                    /* k1  I 项，旧模型没有 */
    0.0f,                    /* k2  w 项，旧模型没有 */
    0.0171687401695532f,     /* k3  I*w（原 k0） */
    0.12785662198126574f,    /* k4  I^2（原 k2） */
    0.00004319062337601348f, /* k5  w^2（原 k1） */
};

/**
 * @brief 默认姿态功率倍率
 * @note  进攻/防御姿态压低到 0.4，移动姿态放宽到 1.2。
 *        这是战术取舍，不是规则要求，按队伍需求改。
 */
const PostureScale kDefaultPostureScale = {
    0.40f, /* attack  */
    0.40f, /* defence */
    1.20f, /* move    */
    1.00f, /* unknown */
};

/* ------------------------------------------------------------------------- */
/* 内部函数                                                                 */
/* ------------------------------------------------------------------------- */

/**
  * @brief          限幅
  * @note           本模块刻意不依赖 alg_basic（那会拉进 CMSIS-DSP），
  *                 只用最基础的 math.h，方便脱离硬件做数值验证。
  */
static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 取当前姿态对应的功率倍率
static float postureScale(const PowerController *ctrl) {
    switch (ctrl->posture) {
        case SentryPosture::Attack:
            return ctrl->posture_scale.attack;
        case SentryPosture::Defence:
            return ctrl->posture_scale.defence;
        case SentryPosture::Move:
            return ctrl->posture_scale.move;
        default:
            return ctrl->posture_scale.unknown;
    }
}

// 更新所有电机的衰前功率估计和总功率
static void updateEstimates(PowerController *ctrl, const MotorCommand *motors, uint8_t count) {
    ctrl->origin_power = 0.0f;

    for (uint8_t i = 0; i < count; i++) {
        ctrl->estimate[i] = powerEstimate(
            &ctrl->coeff,
            motors[i].target_current,
            motors[i].velocity
        );
        ctrl->origin_power += ctrl->estimate[i];
    }
}

#if POWER_CONTROL_USE_POWER_DECAY

// 优先挑和目标同向的根，保持驱动方向不变；两个根方向都不匹配时取绝对值小
static float pickRoot(float r1, float r2, float target) {
    if ((target > 0.0f && r1 > 0.0f) || (target < 0.0f && r1 < 0.0f)) {
        return r1;
    }
    if ((target > 0.0f && r2 > 0.0f) || (target < 0.0f && r2 < 0.0f)) {
        return r2;
    }
    return (fabsf(r1) < fabsf(r2)) ? r1 : r2;
}

// 反解单个电机衰减后的电流
static float computePowerDecayCurrent(
    const PowerController *ctrl,
    const MotorCommand *motor,
    float estimate
) {
    const float target = motor->target_current;
    const float limit = ctrl->current_limit;

    if (ctrl->factor >= 1.0f) {
        return clampf(target, -limit, limit);
    }

    if (estimate <= 0.0f) {
        return clampf(target, -limit, limit);
    }

    /* 解 k4*I^2 + (k1 + k3*w)*I
          + (k0 + k2*w + k5*w^2 - ρ*P_origin) = 0 */
    const float a2 = ctrl->coeff.k4;
    const float b2 = ctrl->coeff.k1 + ctrl->coeff.k3 * motor->velocity;
    const float c2 = ctrl->coeff.k0
                   + ctrl->coeff.k2 * motor->velocity
                   + ctrl->coeff.k5 * motor->velocity * motor->velocity
                   - ctrl->factor * estimate;

    float solved;
    if (a2 > 0.0f) {
        const float delta = b2 * b2 - 4.0f * a2 * c2;
        if (delta > 0.0f) {
            const float h = sqrtf(delta);
            const float r1 = (-b2 + h) / (2.0f * a2);
            const float r2 = (-b2 - h) / (2.0f * a2);
            solved = pickRoot(r1, r2, target);
        } else {
            solved = -b2 / (2.0f * a2);
        }
    } else {
        if (fabsf(b2) < 1e-6f) {
            return clampf(target, -limit, limit);
        }
        solved = -c2 / b2;
    }

    if (solved * target <= 0.0f) {
        solved = 0.0f;
    }

    if (fabsf(solved) > fabsf(target) && (solved * target) >= 0.0f) {
        solved = target;
    }

    return clampf(solved, -limit, limit);
}

/**
  * @brief          第一版：逐电机按功率比例衰减
  * @param[out]     ctrl: 控制器结构指针
  * @param[in]      motors: 本轮电机指令
  * @param[in]      count: 电机数量
  * @param[in]      budget: 本轮预算，W
  * @retval         none
  * @note           先算功率因子 ρ，再逐个反解电流，结果写入 output[]。
  */
static void solvePowerDecay(
    PowerController *ctrl,
    const MotorCommand *motors,
    uint8_t count,
    float budget
) {
    float positive_power = 0.0f;
    float recovery_power = 0.0f;

    for (uint8_t i = 0; i < count; i++) {
        if (ctrl->estimate[i] > 0.0f) {
            positive_power += ctrl->estimate[i];
        } else {
            recovery_power -= ctrl->estimate[i];
        }
    }

    const float usable_power = budget + recovery_power;

    if (positive_power <= 0.0f || positive_power <= usable_power) {
        ctrl->factor = 1.0f;
    } else {
        ctrl->factor = usable_power / positive_power;
    }
    ctrl->factor = clampf(ctrl->factor, 0.0f, 1.0f);

    ctrl->consume_power = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        const float estimate = ctrl->estimate[i];
        ctrl->consume_power += (estimate > 0.0f) ? ctrl->factor * estimate : estimate;
    }

    for (uint8_t i = 0; i < count; i++) {
        ctrl->output[i] = computePowerDecayCurrent(ctrl, &motors[i], ctrl->estimate[i]);
    }
}

#else

// 解电流衰减系数 η
static float solveCurrentFactor(float a, float b, float c, float budget) {
    const float e = c - budget;

    /* k2 异常或所有指令电流都为 0 时退化成一元一次方程 */
    if (!(a > 0.0f)) {
        if (fabsf(b) < 1e-6f) {
            return 0.0f;
        }
        const float eta = -e / b;
        return (eta >= 0.0f && eta <= 1.0f) ? eta : 0.0f;
    }

    const float delta = b * b - 4.0f * a * e;
    if (delta < 0.0f) {
        /* 无实根：再怎么衰减也压不到预算内，直接停转 */
        return 0.0f;
    }

    const float h = sqrtf(delta);
    const float eta1 = (-b - h) / (2.0f * a);
    const float eta2 = (-b + h) / (2.0f * a);

    float eta = 0.0f;
    bool found = false;

    if (eta1 >= 0.0f && eta1 <= 1.0f) {
        eta = eta1;
        found = true;
    }
    if (eta2 >= 0.0f && eta2 <= 1.0f) {
        eta = eta2; /* 两个根都有效时取较大的 */
        found = true;
    }

    return found ? eta : 0.0f;
}

// 解总功率方程得到 η，再统一乘到每个电流上，结果写入 output[]。
static void solveCurrentDecay(
    PowerController *ctrl,
    const MotorCommand *motors,
    uint8_t count,
    float budget
) {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;

    for (uint8_t i = 0; i < count; i++) {
        const float current = motors[i].target_current;
        const float velocity = motors[i].velocity;

        a += ctrl->coeff.k4 * current * current;
        b += ctrl->coeff.k1 * current + ctrl->coeff.k3 * current * velocity;
        c += ctrl->coeff.k0 + ctrl->coeff.k2 * velocity
           + ctrl->coeff.k5 * velocity * velocity;
    }

    if (ctrl->origin_power <= budget) {
        ctrl->factor = 1.0f;
    } else {
        ctrl->factor = solveCurrentFactor(a, b, c, budget);
    }
    ctrl->factor = clampf(ctrl->factor, 0.0f, 1.0f);

    /* 衰减后预计总功率（含发电回流，可能为负） */
    ctrl->consume_power = a * ctrl->factor * ctrl->factor
                        + b * ctrl->factor
                        + c;

    for (uint8_t i = 0; i < count; i++) {
        ctrl->output[i] = clampf(
            ctrl->factor * motors[i].target_current,
            -ctrl->current_limit,
            ctrl->current_limit
        );
    }
}

#endif /* POWER_CONTROL_USE_POWER_DECAY */

/* ------------------------------------------------------------------------- */
/* API                                                                */
/* ------------------------------------------------------------------------- */

void powerControllerInit(
    PowerController *ctrl,
    const ModelCoefficients *coeff,
    float referee_power_limit
) {
    if (ctrl == nullptr) {
        return;
    }

    memset(ctrl, 0, sizeof(PowerController));

    ctrl->coeff = (coeff != nullptr) ? *coeff : kDefaultC620Coefficients;
    ctrl->posture_scale = kDefaultPostureScale;

    /* 裁判系统没给上限时的兜底值，45W 是哨兵常见的基础上限 */
    ctrl->referee_power_limit = (referee_power_limit > 0.0f) ? referee_power_limit : 45.0f;

    /* 留 10% 余量，避免估算误差顶到上限 */
    ctrl->margin = 0.9f;

    ctrl->factor = 1.0f;
    ctrl->current_limit = 20.0f; /* C620 额定 20A */
    ctrl->posture = SentryPosture::Unknown;
}

void powerControllerSetCoefficients(PowerController *ctrl, const ModelCoefficients *coeff) {
    if (ctrl == nullptr || coeff == nullptr) {
        return;
    }
    ctrl->coeff = *coeff;
}

void powerControllerSetRefereeLimit(PowerController *ctrl, float limit) {
    if (ctrl == nullptr || limit <= 0.0f) {
        return;
    }
    ctrl->referee_power_limit = limit;
}

void powerControllerSetPosture(PowerController *ctrl, SentryPosture posture) {
    if (ctrl == nullptr) {
        return;
    }
    ctrl->posture = posture;
}

void powerControllerSetCurrentLimit(PowerController *ctrl, float limit) {
    if (ctrl == nullptr || limit <= 0.0f) {
        return;
    }
    ctrl->current_limit = limit;
}

float powerEstimate(const ModelCoefficients *coeff, float current, float velocity) {
    if (coeff == nullptr) {
        return 0.0f;
    }

    return coeff->k0
         + coeff->k1 * current
         + coeff->k2 * velocity
         + coeff->k3 * current * velocity
         + coeff->k4 * current * current
         + coeff->k5 * velocity * velocity;
}

void powerControllerUpdate(PowerController *ctrl, const MotorCommand *motors, uint8_t count) {
    if (ctrl == nullptr) {
        return;
    }
    if (motors == nullptr) {
        count = 0;
    }
    if (count > POWER_CONTROL_MAX_MOTOR) {
        count = POWER_CONTROL_MAX_MOTOR;
    }
    ctrl->count = count;

    updateEstimates(ctrl, motors, count);

    const float budget = ctrl->referee_power_limit * postureScale(ctrl) * ctrl->margin;
    ctrl->available_power = budget;

#if POWER_CONTROL_USE_POWER_DECAY
    solvePowerDecay(ctrl, motors, count, budget);
#else
    solveCurrentDecay(ctrl, motors, count, budget);
#endif
}

float powerControllerLimitCurrent(const PowerController *ctrl, uint8_t index) {
    if (ctrl == nullptr || index >= ctrl->count) {
        return 0.0f;
    }

    return ctrl->output[index];
}

float powerControllerAvailablePower(const PowerController *ctrl) {
    return (ctrl != nullptr) ? ctrl->available_power : 0.0f;
}

float powerControllerOriginPower(const PowerController *ctrl) {
    return (ctrl != nullptr) ? ctrl->origin_power : 0.0f;
}

float powerControllerConsumePower(const PowerController *ctrl) {
    return (ctrl != nullptr) ? ctrl->consume_power : 0.0f;
}

float powerControllerFactor(const PowerController *ctrl) {
    return (ctrl != nullptr) ? ctrl->factor : 1.0f;
}

} // namespace power_control

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
