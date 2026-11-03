/**
 * @file alg_force.cpp
 * @brief 多执行器力分配与执行器模型补偿实现。
 * @copyright WHU-LuojiaFox (c) 2026
 *
 * 每个周期先通过分配矩阵把广义力映射为各执行器输出轴力矩，再叠加惯性、
 * 摩擦、前馈和重力补偿，最后转换为驱动控制量。
 */

#include "alg_force.hpp"

#include <algorithm>
#include <cmath>

#include "alg_basic.hpp"
#include "mat_matrix.hpp"
#include "prf_cycle_counter.hpp"

namespace {

bool allFinite(const float* values, uint8_t count)
{
    if (values == nullptr) {
        return false;
    }
    for (uint8_t index = 0U; index < count; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

void clearOutput(float* torque, float* output, uint8_t count)
{
    std::fill_n(torque, count, 0.0f);
    std::fill_n(output, count, 0.0f);
}

bool distributionFullRank(
    const float* matrix,
    uint8_t actuator_count,
    uint8_t dof_count
)
{
    matrix::View view {};
    matrix::init(
        &view,
        actuator_count,
        dof_count,
        const_cast<float*>(matrix)
    );
    double work[ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX] {};
    uint8_t value = 0U;
    return matrix::rank(&view, work, ALG_FORCE_DOF_MAX, &value)
        == ARM_MATH_SUCCESS
        && value == dof_count;
}

bool parameterValid(const ForceActuatorParam& param)
{
    const float output_factor =
        param.torque_constant * param.reduction_ratio;
    return std::isfinite(param.torque_constant)
        && param.torque_constant > 0.0f
        && std::isfinite(param.reduction_ratio)
        && param.reduction_ratio > 0.0f
        && (param.direction == 1 || param.direction == -1)
        && std::isfinite(param.torque_max)
        && param.torque_max > 0.0f
        && std::isfinite(param.inertia)
        && param.inertia >= 0.0f
        && std::isfinite(param.viscous_friction)
        && param.viscous_friction >= 0.0f
        && std::isfinite(param.coulomb_friction)
        && param.coulomb_friction >= 0.0f
        && std::isfinite(param.gravity_amplitude)
        && param.gravity_amplitude >= 0.0f
        && std::isfinite(param.gravity_offset)
        && std::isfinite(output_factor)
        && output_factor > 0.0f;
}

} // namespace

/**
 * @brief 初始化求解器、换算系数和默认分配矩阵。
 */
bool ForceSolver::init(
    const ForceActuatorParam* parameters,
    uint8_t actuator_count,
    uint8_t dof_count,
    float dt
) {
#if LFCORE_PROFILE
    profiling::initCycleCounter();
#endif
    this->enabled = false;
    this->actuator_num = 0U;
    this->dof_num = 0U;
    this->distribution_valid = false;
    this->target_valid = false;
    this->feedback_valid = false;
    this->feedforward_valid = true;
    this->saturation_scale = 1.0f;

    if (parameters == nullptr || actuator_count == 0U
        || actuator_count > ALG_FORCE_ACTUATOR_MAX
        || dof_count == 0U || dof_count > ALG_FORCE_DOF_MAX
        || actuator_count < dof_count || !std::isfinite(dt) || dt <= 0.0f) {
        clearOutput(
            this->torque,
            this->out,
            ALG_FORCE_ACTUATOR_MAX
        );
        return false;
    }

    const float candidate_inverse_dt = 1.0f / dt;
    if (!std::isfinite(candidate_inverse_dt)) {
        clearOutput(
            this->torque,
            this->out,
            ALG_FORCE_ACTUATOR_MAX
        );
        return false;
    }

    for (uint8_t index = 0U; index < actuator_count; ++index) {
        if (!parameterValid(parameters[index])
            || !std::isfinite(
                parameters[index].inertia * candidate_inverse_dt
            )) {
            clearOutput(
                this->torque,
                this->out,
                ALG_FORCE_ACTUATOR_MAX
            );
            return false;
        }
    }

    this->actuator_num = actuator_count;
    this->dof_num = dof_count;
    this->inverse_dt = candidate_inverse_dt;

    // 清空所有矩阵元素，然后只在有效执行器/自由度范围内设置单位矩阵。
    std::fill_n(this->distribution, ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX, 0.0f);
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        this->param[index] = parameters[index];

        // torque / (Kt * ratio) 将输出轴力矩换算为驱动控制量。
        // direction 用于适配执行器安装方向。
        const float denominator =
            parameters[index].torque_constant * parameters[index].reduction_ratio;
        this->output_scale[index] = parameters[index].direction / denominator;

        this->torque_limit[index] = parameters[index].torque_max;

        // 由速度变化估算加速度时，inertia * dv / dt 即为补偿力矩。
        this->acceleration_gain[index] = parameters[index].inertia * this->inverse_dt;
        if (index < this->dof_num) {
            this->distribution[index * this->dof_num + index] = 1.0f;
        }
    }

    // 清空上一周期目标、反馈、滤波器和输出，保证从确定的零状态启动。
    this->distribution_valid = true;
    this->reset();
    return true;
}

/**
 * @brief 复制调用方提供的行优先分配矩阵。
 */
bool ForceSolver::setDistribution(const float* matrix) {
    const uint8_t count = this->actuator_num * this->dof_num;
    if (this->actuator_num == 0U || this->dof_num == 0U
        || !allFinite(matrix, count)
        || !distributionFullRank(matrix, this->actuator_num, this->dof_num)) {
        std::fill_n(
            this->distribution,
            ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX,
            0.0f
        );
        this->distribution_valid = false;
        return false;
    }
    // 仅复制有效执行器 x 有效自由度区域，不复制固定数组中的填充部分。
    std::copy_n(matrix, count, this->distribution);
    this->distribution_valid = true;
    return true;
}

/**
 * @brief 更新本周期广义力目标。
 */
bool ForceSolver::setTarget(const float* generalized_force) {
    if (!allFinite(generalized_force, this->dof_num)) {
        std::fill_n(
            this->target_generalized_force,
            ALG_FORCE_DOF_MAX,
            0.0f
        );
        this->target_valid = false;
        return false;
    }
    std::copy_n(generalized_force, this->dof_num, this->target_generalized_force);
    this->target_valid = true;
    return true;
}

/**
 * @brief 更新执行器位置反馈和经过低通滤波的速度反馈。
 */
bool ForceSolver::setNow(const float* positions, const float* velocities) {
    if ((positions != nullptr && !allFinite(positions, this->actuator_num))
        || (velocities != nullptr && !allFinite(velocities, this->actuator_num))) {
        this->feedback_valid = false;
        return false;
    }

    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        // 位置仅用于重力补偿；为空时保留上一次值。
        if (positions != nullptr) {
            this->position[index] = positions[index];
        }

        if (velocities != nullptr) {
            // 首帧直接用当前速度初始化滤波器，避免从零缓慢爬升。
            if (!this->velocity_filter_ready[index]) {
                this->velocity[index] = velocities[index];
                // 首帧没有可靠的历史速度，禁用本周期加速度补偿；
                // 否则重新使能时会把当前速度误判为一步内的巨大加速度。
                this->pre_velocity[index] = velocities[index];
                this->velocity_filter_ready[index] = true;
            }

            // 一阶低通：velocity = alpha * 旧值 + (1-alpha) * 新反馈。
            this->velocity[index] = this->velocity_filter_alpha * this->velocity[index]
                + this->velocity_filter_weight * velocities[index];
        }
    }
    this->feedback_valid = true;
    return true;
}

/**
 * @brief 更新各执行器的前馈力矩。
 */
bool ForceSolver::setFeedforwardTorque(const float* torques) {
    if (!allFinite(torques, this->actuator_num)) {
        std::fill_n(
            this->feedforward_torque,
            ALG_FORCE_ACTUATOR_MAX,
            0.0f
        );
        this->feedforward_valid = false;
        return false;
    }
    std::copy_n(torques, this->actuator_num, this->feedforward_torque);
    this->feedforward_valid = true;
    return true;
}

/**
 * @brief 设置速度低通滤波系数并同步计算新值权重。
 */
bool ForceSolver::setVelocityFilterAlpha(float alpha) {
    if (!std::isfinite(alpha)) {
        return false;
    }
    this->velocity_filter_alpha = basic_math::constrain(alpha, 0.0f, 1.0f);
    this->velocity_filter_weight = 1.0f - this->velocity_filter_alpha;
    return true;
}

/**
 * @brief 使能或停用求解器。
 */
void ForceSolver::setEnable(bool enable) {
    this->enabled = enable;

    // 停用时立即清空历史状态，重新使能后不会复用旧的积分/速度/输出。
    if (!enable) {
        this->reset();
    }
}

/**
 * @brief 执行一次力分配与补偿计算。
 */
void ForceSolver::timCalculatePeriodElapsedCallback() {
#if LFCORE_PROFILE
    profiling::CycleScope cycle_scope { profiling::force_cycle_stats };
#endif

    // 未使能、没有执行器或任一必要输入无效时保持零输出。
    if (!this->enabled || this->actuator_num == 0
        || !this->distribution_valid || !this->target_valid
        || !this->feedback_valid || !this->feedforward_valid) {
        clearOutput(this->torque, this->out, this->actuator_num);
        std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
        this->saturation_scale = 0.0f;
        return;
    }

    // 先做 actuator_num x dof_num 矩阵乘法，得到每个执行器的基础输出轴力矩。
    // 小列数内核避免构造矩阵描述符和调用通用 DSP 接口。
    if (matrix::multiplyVector(
            this->actuator_num,
            this->dof_num,
            this->distribution,
            this->target_generalized_force,
            this->torque
        )
        != ARM_MATH_SUCCESS)
    {
        // 矩阵计算失败时禁止沿用旧输出，保证故障状态为零力矩。
        clearOutput(this->torque, this->out, this->actuator_num);
        std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
        this->saturation_scale = 0.0f;
        return;
    }

    if (!allFinite(this->torque, this->actuator_num)) {
        clearOutput(this->torque, this->out, this->actuator_num);
        std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
        this->saturation_scale = 0.0f;
        return;
    }

    // 对每个执行器叠加模型补偿，再按照机械参数换算为驱动控制量。
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        const auto& model = this->param[index];

        // 基础力矩 + 惯性补偿 + 粘性摩擦 + 库仑摩擦 + 外部前馈。
        float value = this->torque[index]
            + this->acceleration_gain[index] * (this->velocity[index] - this->pre_velocity[index])
            + model.viscous_friction * this->velocity[index]
            + model.coulomb_friction * basic_math::sign(this->velocity[index])
            + this->feedforward_torque[index];

        // 重力补偿按执行器位置的正弦/余弦模型叠加；幅值为 0 时跳过。
        if (model.gravity_amplitude != 0.0f) {
            value += model.gravity_amplitude
                * basic_math::cos(this->position[index] - model.gravity_offset);
        }

        if (!std::isfinite(value)) {
            clearOutput(this->torque, this->out, this->actuator_num);
            std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
            this->saturation_scale = 0.0f;
            return;
        }
        this->torque[index] = value;

        // 保存本周期速度，用于下一周期估算加速度。
        this->pre_velocity[index] = this->velocity[index];
    }

    // 任一执行器超限时统一缩放四轮力矩，保持轮间比例和整车力方向。
    float scale = 1.0f;
    for (uint8_t index = 0U; index < this->actuator_num; ++index) {
        if (std::fabs(this->torque[index]) > this->torque_limit[index]) {
            scale = std::fmin(
                scale,
                this->torque_limit[index] / std::fabs(this->torque[index])
            );
        }
    }
    scale = basic_math::constrain(scale, 0.0f, 1.0f);
    this->saturation_scale = scale;

    for (uint8_t index = 0U; index < this->actuator_num; ++index) {
        this->torque[index] *= scale;
        this->out[index] = this->torque[index] * this->output_scale[index];
    }
}

/**
 * @brief 清空动态状态，但保留初始化参数和分配矩阵。
 */
void ForceSolver::reset() {
    std::fill_n(this->target_generalized_force, ALG_FORCE_DOF_MAX, 0.0f);
    std::fill_n(this->feedforward_torque, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    std::fill_n(this->position, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    std::fill_n(this->velocity, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    std::fill_n(this->pre_velocity, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    std::fill_n(this->velocity_filter_ready, ALG_FORCE_ACTUATOR_MAX, false);
    std::fill_n(this->torque, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    std::fill_n(this->out, ALG_FORCE_ACTUATOR_MAX, 0.0f);
    this->saturation_scale = 1.0f;
    this->target_valid = false;
    this->feedback_valid = false;
    this->feedforward_valid = true;
}

/**
 * @brief 构建四轮全向底盘几何分配矩阵。
 */
bool ForceSolver::buildOmniDistribution(
    float wheel_radius,
    float wheel_center_distance,
    float* matrix
) {
    // 输出缓冲区为空或机械尺度非正/非有限时无法构建有效矩阵。
    if (matrix == nullptr
        || !std::isfinite(wheel_radius) || wheel_radius <= 0.0f
        || !std::isfinite(wheel_center_distance)
        || wheel_center_distance <= 0.0f) {
        return false;
    }

    // 四个全向轮相对车体坐标系的固定安装方向。
    constexpr float WHEEL_X[4] = { 0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f };
    constexpr float WHEEL_Y[4] = { -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f };

    // 平移项和旋转项分别将广义力 [Fx, Fy, Mz] 映射为轮输出轴力矩。
    const float translation_scale = wheel_radius * 0.5f;
    const float rotation_scale = -wheel_radius / (4.0f * wheel_center_distance);
    if (!std::isfinite(translation_scale) || translation_scale <= 0.0f
        || !std::isfinite(rotation_scale) || rotation_scale >= 0.0f) {
        return false;
    }

    // 按执行器行、[Fx, Fy, Mz] 列填充 4x3 行优先矩阵。
    for (uint8_t index = 0; index < 4; ++index) {
        matrix[index * 3] = WHEEL_X[index] * translation_scale;
        matrix[index * 3 + 1] = WHEEL_Y[index] * translation_scale;
        matrix[index * 3 + 2] = rotation_scale;
    }
    return true;
}
