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

/**
 * @brief 初始化求解器、换算系数和默认分配矩阵。
 */
void ForceSolver::init(
    const ForceActuatorParam* parameters,
    uint8_t actuator_count,
    uint8_t dof_count,
    float dt
) {
#if LFCORE_PROFILE
    profiling::initCycleCounter();
#endif
    // 限制执行器和自由度数量，避免写入固定长度内部数组时越界。
    this->actuator_num =
        parameters == nullptr ? 0 : std::min<uint8_t>(actuator_count, ALG_FORCE_ACTUATOR_MAX);
    this->dof_num = basic_math::constrain<uint8_t>(dof_count, 1, ALG_FORCE_DOF_MAX);

    // 非法或非正控制周期回退到 1 ms，避免除零和加速度补偿异常。
    this->inverse_dt = 1.0f / (std::isfinite(dt) && dt > 0.0f ? dt : 0.001f);
    this->enabled = false;

    // 清空所有矩阵元素，然后只在有效执行器/自由度范围内设置单位矩阵。
    std::fill_n(this->distribution, ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX, 0.0f);
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        this->param[index] = parameters[index];

        // torque / (Kt * ratio) 将输出轴力矩换算为驱动控制量。
        // direction 用于适配执行器安装方向。
        const float denominator =
            parameters[index].torque_constant * parameters[index].reduction_ratio;
        this->output_scale[index] =
            denominator != 0.0f ? parameters[index].direction / denominator : 0.0f;

        // 力矩上限使用幅值，允许调用方以负值配置上限。
        this->torque_limit[index] = basic_math::abs(parameters[index].torque_max);

        // 由速度变化估算加速度时，inertia * dv / dt 即为补偿力矩。
        this->acceleration_gain[index] = parameters[index].inertia * this->inverse_dt;
        if (index < this->dof_num) {
            this->distribution[index * this->dof_num + index] = 1.0f;
        }
    }

    // 清空上一周期目标、反馈、滤波器和输出，保证从确定的零状态启动。
    this->reset();
}

/**
 * @brief 复制调用方提供的行优先分配矩阵。
 */
void ForceSolver::setDistribution(const float* matrix) {
    if (matrix != nullptr) {
        // 仅复制有效执行器 x 有效自由度区域，不复制固定数组中的填充部分。
        std::copy_n(matrix, this->actuator_num * this->dof_num, this->distribution);
    }
}

/**
 * @brief 更新本周期广义力目标。
 */
void ForceSolver::setTarget(const float* generalized_force) {
    if (generalized_force != nullptr) {
        std::copy_n(generalized_force, this->dof_num, this->target_generalized_force);
    }
}

/**
 * @brief 更新执行器位置反馈和经过低通滤波的速度反馈。
 */
void ForceSolver::setNow(const float* positions, const float* velocities) {
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        // 位置仅用于重力补偿；为空时保留上一次值。
        if (positions != nullptr) {
            this->position[index] = positions[index];
        }

        if (velocities != nullptr) {
            // 首帧直接用当前速度初始化滤波器，避免从零缓慢爬升。
            if (!this->velocity_filter_ready[index]) {
                this->velocity[index] = velocities[index];
                this->velocity_filter_ready[index] = true;
            }

            // 一阶低通：velocity = alpha * 旧值 + (1-alpha) * 新反馈。
            this->velocity[index] = this->velocity_filter_alpha * this->velocity[index]
                + this->velocity_filter_weight * velocities[index];
        }
    }
}

/**
 * @brief 更新各执行器的前馈力矩。
 */
void ForceSolver::setFeedforwardTorque(const float* torques) {
    if (torques != nullptr) {
        std::copy_n(torques, this->actuator_num, this->feedforward_torque);
    }
}

/**
 * @brief 设置速度低通滤波系数并同步计算新值权重。
 */
void ForceSolver::setVelocityFilterAlpha(float alpha) {
    this->velocity_filter_alpha = basic_math::constrain(alpha, 0.0f, 1.0f);
    this->velocity_filter_weight = 1.0f - this->velocity_filter_alpha;
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

    // 未使能或没有执行器时保持零输出；仍记录速度供重新使能时参考。
    if (!this->enabled || this->actuator_num == 0) {
        std::fill_n(this->torque, this->actuator_num, 0.0f);
        std::fill_n(this->out, this->actuator_num, 0.0f);
        std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
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
        std::fill_n(this->torque, this->actuator_num, 0.0f);
        std::fill_n(this->out, this->actuator_num, 0.0f);
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

        // 补偿后的总力矩必须限制在配置的机械安全范围内。
        this->torque[index] =
            basic_math::constrain(value, -this->torque_limit[index], this->torque_limit[index]);

        // 力矩转换为驱动控制量，并应用减速比和执行器方向。
        this->out[index] = this->torque[index] * this->output_scale[index];

        // 保存本周期速度，用于下一周期估算加速度。
        this->pre_velocity[index] = this->velocity[index];
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
}

/**
 * @brief 构建四轮全向底盘几何分配矩阵。
 */
bool ForceSolver::buildOmniDistribution(
    float wheel_radius,
    float wheel_center_distance,
    float* matrix
) {
    // 输出缓冲区为零或机械尺度为零时无法构建有效矩阵。
    if (matrix == nullptr || wheel_center_distance == 0.0f || wheel_radius == 0.0f) {
        return false;
    }

    // 四个全向轮相对车体坐标系的固定安装方向。
    constexpr float WHEEL_X[4] = { 0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f };
    constexpr float WHEEL_Y[4] = { -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f };

    // 平移项和旋转项分别将广义力 [Fx, Fy, Mz] 映射为轮输出轴力矩。
    const float translation_scale = wheel_radius * 0.5f;
    const float rotation_scale = -wheel_radius / (4.0f * wheel_center_distance);

    // 按执行器行、[Fx, Fy, Mz] 列填充 4x3 行优先矩阵。
    for (uint8_t index = 0; index < 4; ++index) {
        matrix[index * 3] = WHEEL_X[index] * translation_scale;
        matrix[index * 3 + 1] = WHEEL_Y[index] * translation_scale;
        matrix[index * 3 + 2] = rotation_scale;
    }
    return true;
}
