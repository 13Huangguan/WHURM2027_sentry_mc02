/** @file alg_power_control.cpp
 * @brief 四参数电机模型下的统一电流衰减与逐电机功率分配。
 * @copyright WHU-LuojiaFox (c) 2026
 */
#include "alg_power_control.hpp"
#include "alg_basic.hpp"
#include <float.h>
#include <math.h>
#include <string.h>

namespace power_control {

// 只删除原模型的两个零项，沿用原标定数值，没有重新拟合或改变单位。
const ModelCoefficients kDefaultC620Coefficients = {
    1.598658371452403f,       // k0: 单电机常数项
    0.0171687401695532f,      // k3: I*w
    0.12785662198126574f,     // k4: I^2
    0.00004319062337601348f,  // k5: w^2
};
const PostureScale kDefaultPostureScale = {0.40f, 0.40f, 1.00f, 1.00f};

namespace {
struct MotorModel {
    float target;   // 调用方已限幅的电流，本模块不再截断
    float b;        // k3*w
    float c;        // k0+k5*w^2
    float minimum;  // 当前方法可达的最小功率所对应电流
};

bool validCoefficients(const ModelCoefficients& c)
{
    return isfinite(c.k0) && isfinite(c.k3) && isfinite(c.k4) && isfinite(c.k5)
        && c.k0 >= 0.0f && c.k4 >= 0.0f && c.k5 >= 0.0f;
}
float postureScale(const PowerController& ctrl)
{
    switch (ctrl.posture) {
        case SentryPosture::Attack: return ctrl.posture_scale.attack;
        case SentryPosture::Defence: return ctrl.posture_scale.defence;
        case SentryPosture::Move: return ctrl.posture_scale.move;
        default: return ctrl.posture_scale.unknown;
    }
}
float polynomial(float a, float b, float c, float x)
{
    return (a * x + b) * x + c;
}
float motorPower(float a, const MotorModel& motor, float current)
{
    return polynomial(a, motor.b, motor.c, current);
}

// 凸二次函数在 [0,1] 内的最小点。先比较端点导数，避免 b/a 溢出。
float minimumScale(float a, float b)
{
    if (b >= 0.0f) { return 0.0f; }
    if (a == 0.0f || a <= -0.5f * b) { return 1.0f; }
    return (-0.5f * b) / a;
}

// 在 [0,1] 内求满足 a*x^2+b*x+c<=budget 的最大 x。
// 无可行解时返回最小功率点，由最终复核显式标记不可达。
float solveScale(float a, float b, float c, float budget)
{
    if (polynomial(a, b, c, 1.0f) <= budget) { return 1.0f; }
    const float minimum = minimumScale(a, b);
    if (polynomial(a, b, c, minimum) >= budget) { return minimum; }
    const auto roots = basic_math::solveQuadratic(a, b, c - budget);
    if (roots.status != basic_math::QuadraticStatus::OneRealRoot
        && roots.status != basic_math::QuadraticStatus::TwoRealRoots) {
        return minimum;
    }
    // 凸功率曲线的右根对应最大可行比例；区间与预算策略由 power 负责。
    const float root = basic_math::constrain(roots.upper, minimum, 1.0f);
    // 朝低功率方向留少量浮点余量，减少最终复核触发迭代修正的次数。
    return fmaxf(minimum, root - 4.0f * FLT_EPSILON * basic_math::abs(root));
}

void resetCycle(PowerController& ctrl)
{
    memset(ctrl.output, 0, sizeof(ctrl.output));
    memset(ctrl.estimate, 0, sizeof(ctrl.estimate));
    ctrl.available_power = 0.0f;
    ctrl.origin_power = 0.0f;
    ctrl.consume_power = 0.0f;
    ctrl.minimum_power = 0.0f;
    ctrl.factor = 0.0f;
    ctrl.budget_feasible = false;
    ctrl.status = PowerControlStatus::InvalidInput;
}
float recomputePower(const PowerController& ctrl, const MotorModel* motors)
{
    float result = 0.0f;
    for (uint8_t i = 0; i < ctrl.count; ++i) {
        result += motorPower(ctrl.coeff.k4, motors[i], ctrl.output[i]);
    }
    return result;
}

// 仅在最终浮点复核超预算时触发：沿最低功率点到候选输出作有界修正。
// 直接用最终电流复算，不以理论分配值冒充实际输出的模型功率。
float verifyOutput(PowerController& ctrl, const MotorModel* motors)
{
    ctrl.consume_power = recomputePower(ctrl, motors);
    if (ctrl.consume_power <= ctrl.available_power) { return 1.0f; }
    float minimum_power = 0.0f;
    float delta[POWER_CONTROL_MAX_MOTOR];
    for (uint8_t i = 0; i < ctrl.count; ++i) {
        minimum_power += motorPower(ctrl.coeff.k4, motors[i], motors[i].minimum);
        delta[i] = ctrl.output[i] - motors[i].minimum;
    }
    if (minimum_power > ctrl.available_power) {
        for (uint8_t i = 0; i < ctrl.count; ++i) { ctrl.output[i] = motors[i].minimum; }
        ctrl.consume_power = minimum_power;
        return 0.0f;
    }
    float low = 0.0f;
    float high = 1.0f;
    // 最多 24 次，只处理舍入误差；普通周期不进入此分支。
    for (uint8_t iteration = 0; iteration < 24U; ++iteration) {
        const float mid = low + 0.5f * (high - low);
        float power = 0.0f;
        for (uint8_t i = 0; i < ctrl.count; ++i) {
            const float current = motors[i].minimum + mid * delta[i];
            power += motorPower(ctrl.coeff.k4, motors[i], current);
        }
        if (power <= ctrl.available_power) { low = mid; } else { high = mid; }
    }
    for (uint8_t i = 0; i < ctrl.count; ++i) {
        ctrl.output[i] = motors[i].minimum + low * delta[i];
    }
    ctrl.consume_power = recomputePower(ctrl, motors);
    return low;
}
} // namespace

void powerControllerInit(PowerController* ctrl, const ModelCoefficients* coeff, float limit)
{
    if (ctrl == nullptr) { return; }
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->coeff = coeff != nullptr ? *coeff : kDefaultC620Coefficients;
    ctrl->posture_scale = kDefaultPostureScale;
    ctrl->referee_power_limit = limit;
    ctrl->margin = 0.9f;
    ctrl->current_limit = 20.0f;
    ctrl->posture = SentryPosture::Unknown;
    ctrl->status = PowerControlStatus::Disabled;
}
void powerControllerSetCoefficients(PowerController* ctrl, const ModelCoefficients* coeff)
{
    if (ctrl != nullptr && coeff != nullptr) { ctrl->coeff = *coeff; }
}
void powerControllerSetPowerLimit(PowerController* ctrl, float limit)
{
    if (ctrl != nullptr) { ctrl->referee_power_limit = limit; }
}
void powerControllerSetRefereeLimit(PowerController* ctrl, float limit)
{
    powerControllerSetPowerLimit(ctrl, limit);
}
void powerControllerSetPosture(PowerController* ctrl, SentryPosture posture)
{
    if (ctrl != nullptr) { ctrl->posture = posture; }
}
void powerControllerSetCurrentLimit(PowerController* ctrl, float limit)
{
    if (ctrl != nullptr) { ctrl->current_limit = limit; }
}
float powerEstimate(const ModelCoefficients* coeff, float current, float velocity)
{
    if (coeff == nullptr) { return 0.0f; }
    const float constant = coeff->k0 + coeff->k5 * velocity * velocity;
    return (coeff->k4 * current + coeff->k3 * velocity) * current + constant;
}

void powerControllerUpdate(PowerController* ctrl, const MotorCommand* motors, uint8_t count)
{
    if (ctrl == nullptr) { return; }
    resetCycle(*ctrl);
    ctrl->count = count <= POWER_CONTROL_MAX_MOTOR ? count : POWER_CONTROL_MAX_MOTOR;
    const float scale = postureScale(*ctrl);
    if (count > POWER_CONTROL_MAX_MOTOR || (count != 0U && motors == nullptr)
        || !validCoefficients(ctrl->coeff) || !isfinite(ctrl->referee_power_limit)
        || ctrl->referee_power_limit < 0.0f || !isfinite(ctrl->current_limit)
        || ctrl->current_limit < 0.0f || !isfinite(ctrl->margin) || ctrl->margin < 0.0f
        || !isfinite(scale) || scale < 0.0f) { return; }
    // 超电允许功率为硬上限，姿态和裕度只能降低，不能将其放大。
    ctrl->available_power = ctrl->referee_power_limit * basic_math::constrain(scale, 0.0f, 1.0f)
        * basic_math::constrain(ctrl->margin, 0.0f, 1.0f);

    MotorModel model[POWER_CONTROL_MAX_MOTOR];
    float a = 0.0f, b = 0.0f, c = 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        if (!isfinite(motors[i].target_current) || !isfinite(motors[i].velocity)
            || basic_math::abs(motors[i].target_current) > ctrl->current_limit) {
            resetCycle(*ctrl);
            return;
        }
        auto& item = model[i];
        item.target = motors[i].target_current;
        item.b = ctrl->coeff.k3 * motors[i].velocity;
        item.c = ctrl->coeff.k0 + ctrl->coeff.k5 * motors[i].velocity * motors[i].velocity;
        item.minimum = 0.0f;
        const float ai = ctrl->coeff.k4 * item.target * item.target;
        const float bi = item.b * item.target;
        const float power = motorPower(ctrl->coeff.k4, item, item.target);
        a += ai;
        b += bi;
        c += item.c;
        ctrl->estimate[i] = power;
        ctrl->origin_power += power;
        if (!isfinite(item.b) || !isfinite(item.c) || !isfinite(ai) || !isfinite(bi)
            || !isfinite(power) || !isfinite(a) || !isfinite(b) || !isfinite(c)
            || !isfinite(ctrl->origin_power)) {
            resetCycle(*ctrl);
            return;
        }
    }
    if (ctrl->available_power == 0.0f || ctrl->current_limit == 0.0f) {
        ctrl->consume_power = c; // 零电流仍有模型中的常数/速度损耗。
        ctrl->minimum_power = c;
        ctrl->budget_feasible = c <= ctrl->available_power;
        ctrl->status = PowerControlStatus::Disabled;
        return;
    }

#if POWER_CONTROL_USE_POWER_DECAY
    // 负功率电机原样保留输入电流；只分配正功率电机的可调部分。
    for (uint8_t i = 0; i < count; ++i) {
        auto& item = model[i];
        if (ctrl->estimate[i] <= 0.0f) {
            item.minimum = item.target;
        } else {
            item.minimum = item.target * minimumScale(
                ctrl->coeff.k4 * item.target * item.target, item.b * item.target);
        }
        ctrl->minimum_power += motorPower(ctrl->coeff.k4, item, item.minimum);
    }
    if (!isfinite(ctrl->minimum_power)) { resetCycle(*ctrl); return; }
    const float adjustable = fmaxf(0.0f, ctrl->origin_power - ctrl->minimum_power);
    if (!isfinite(adjustable)) { resetCycle(*ctrl); return; }
    ctrl->factor = ctrl->origin_power <= ctrl->available_power ? 1.0f
        : adjustable > 0.0f
            ? basic_math::constrain(
                (ctrl->available_power - ctrl->minimum_power) / adjustable, 0.0f, 1.0f)
            : 0.0f;
    for (uint8_t i = 0; i < count; ++i) {
        const auto& item = model[i];
        if (ctrl->estimate[i] <= 0.0f || ctrl->factor == 1.0f) {
            ctrl->output[i] = item.target;
        } else if (ctrl->factor == 0.0f) {
            ctrl->output[i] = item.minimum;
        } else {
            const float floor = motorPower(ctrl->coeff.k4, item, item.minimum);
            const float allocation = floor + ctrl->factor * (ctrl->estimate[i] - floor);
            ctrl->output[i] = item.target * solveScale(
                ctrl->coeff.k4 * item.target * item.target,
                item.b * item.target, item.c, allocation);
        }
    }
    (void)verifyOutput(*ctrl, model);
    ctrl->factor = adjustable > 0.0f
        ? basic_math::constrain(
            (ctrl->consume_power - ctrl->minimum_power) / adjustable, 0.0f, 1.0f) : 1.0f;
#else
    // 同一次缓存循环已得到 A/B/C，不再单独遍历估计和聚合。
    const float minimum = minimumScale(a, b);
    ctrl->factor = ctrl->origin_power <= ctrl->available_power ? 1.0f
        : solveScale(a, b, c, ctrl->available_power);
    for (uint8_t i = 0; i < count; ++i) {
        model[i].minimum = minimum * model[i].target;
        ctrl->minimum_power += motorPower(ctrl->coeff.k4, model[i], model[i].minimum);
        ctrl->output[i] = ctrl->factor * model[i].target;
    }
    const float correction = verifyOutput(*ctrl, model);
    if (correction < 1.0f) {
        ctrl->factor = minimum + correction * (ctrl->factor - minimum);
    }
#endif
    if (!isfinite(ctrl->consume_power) || !isfinite(ctrl->minimum_power)
        || !isfinite(ctrl->factor)) { resetCycle(*ctrl); return; }
    ctrl->budget_feasible = ctrl->consume_power <= ctrl->available_power;
    ctrl->status = !ctrl->budget_feasible ? PowerControlStatus::BudgetInfeasible
        : ctrl->factor < 1.0f ? PowerControlStatus::Limited : PowerControlStatus::Ready;
}

float powerControllerLimitCurrent(const PowerController* ctrl, uint8_t index)
{
    return ctrl != nullptr && index < ctrl->count ? ctrl->output[index] : 0.0f;
}
float powerControllerAvailablePower(const PowerController* ctrl)
{
    return ctrl != nullptr ? ctrl->available_power : 0.0f;
}
float powerControllerOriginPower(const PowerController* ctrl)
{
    return ctrl != nullptr ? ctrl->origin_power : 0.0f;
}
float powerControllerConsumePower(const PowerController* ctrl)
{
    return ctrl != nullptr ? ctrl->consume_power : 0.0f;
}
float powerControllerFactor(const PowerController* ctrl)
{
    return ctrl != nullptr ? ctrl->factor : 0.0f;
}
} // namespace power_control
