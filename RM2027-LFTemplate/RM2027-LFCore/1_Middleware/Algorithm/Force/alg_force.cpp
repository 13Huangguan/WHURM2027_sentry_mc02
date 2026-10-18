// Copyright WHU-LuojiaFox (c) 2026.

#include "alg_force.hpp"

#include <algorithm>
#include <cmath>

#include "alg_basic.hpp"
#include "mat_matrix.hpp"
#include "prf_cycle_counter.hpp"

void ForceSolver::init(
    const ForceActuatorParam* parameters,
    uint8_t actuator_count,
    uint8_t dof_count,
    float dt
) {
#if LFCORE_PROFILE
    profiling::initCycleCounter();
#endif
    this->actuator_num =
        parameters == nullptr ? 0 : std::min<uint8_t>(actuator_count, ALG_FORCE_ACTUATOR_MAX);
    this->dof_num = basic_math::constrain<uint8_t>(dof_count, 1, ALG_FORCE_DOF_MAX);
    this->inverse_dt = 1.0f / (std::isfinite(dt) && dt > 0.0f ? dt : 0.001f);
    this->enabled = false;
    std::fill_n(this->distribution, ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX, 0.0f);
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        this->param[index] = parameters[index];
        const float denominator =
            parameters[index].torque_constant * parameters[index].reduction_ratio;
        this->output_scale[index] =
            denominator != 0.0f ? parameters[index].direction / denominator : 0.0f;
        this->torque_limit[index] = basic_math::abs(parameters[index].torque_max);
        this->acceleration_gain[index] = parameters[index].inertia * this->inverse_dt;
        if (index < this->dof_num) {
            this->distribution[index * this->dof_num + index] = 1.0f;
        }
    }
    this->reset();
}

void ForceSolver::setDistribution(const float* matrix) {
    if (matrix != nullptr) {
        std::copy_n(matrix, this->actuator_num * this->dof_num, this->distribution);
    }
}

void ForceSolver::setTarget(const float* generalized_force) {
    if (generalized_force != nullptr) {
        std::copy_n(generalized_force, this->dof_num, this->target_generalized_force);
    }
}

void ForceSolver::setNow(const float* positions, const float* velocities) {
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        if (positions != nullptr) {
            this->position[index] = positions[index];
        }
        if (velocities != nullptr) {
            if (!this->velocity_filter_ready[index]) {
                this->velocity[index] = velocities[index];
                this->velocity_filter_ready[index] = true;
            }
            this->velocity[index] = this->velocity_filter_alpha * this->velocity[index]
                + this->velocity_filter_weight * velocities[index];
        }
    }
}

void ForceSolver::setFeedforwardTorque(const float* torques) {
    if (torques != nullptr) {
        std::copy_n(torques, this->actuator_num, this->feedforward_torque);
    }
}

void ForceSolver::setVelocityFilterAlpha(float alpha) {
    this->velocity_filter_alpha = basic_math::constrain(alpha, 0.0f, 1.0f);
    this->velocity_filter_weight = 1.0f - this->velocity_filter_alpha;
}

void ForceSolver::setEnable(bool enable) {
    this->enabled = enable;
    if (!enable) {
        this->reset();
    }
}

void ForceSolver::timCalculatePeriodElapsedCallback() {
#if LFCORE_PROFILE
    profiling::CycleScope cycle_scope { profiling::force_cycle_stats };
#endif
    if (!this->enabled || this->actuator_num == 0) {
        std::fill_n(this->torque, this->actuator_num, 0.0f);
        std::fill_n(this->out, this->actuator_num, 0.0f);
        std::copy_n(this->velocity, this->actuator_num, this->pre_velocity);
        return;
    }
    // Small-column kernels avoid descriptor construction and the generic DSP call.
    if (matrix::multiplyVector(
            this->actuator_num,
            this->dof_num,
            this->distribution,
            this->target_generalized_force,
            this->torque
        )
        != ARM_MATH_SUCCESS)
    {
        std::fill_n(this->torque, this->actuator_num, 0.0f);
        std::fill_n(this->out, this->actuator_num, 0.0f);
        return;
    }
    for (uint8_t index = 0; index < this->actuator_num; ++index) {
        const auto& model = this->param[index];
        float value = this->torque[index]
            + this->acceleration_gain[index] * (this->velocity[index] - this->pre_velocity[index])
            + model.viscous_friction * this->velocity[index]
            + model.coulomb_friction * basic_math::sign(this->velocity[index])
            + this->feedforward_torque[index];
        if (model.gravity_amplitude != 0.0f) {
            value += model.gravity_amplitude
                * basic_math::cos(this->position[index] - model.gravity_offset);
        }
        this->torque[index] =
            basic_math::constrain(value, -this->torque_limit[index], this->torque_limit[index]);
        this->out[index] = this->torque[index] * this->output_scale[index];
        this->pre_velocity[index] = this->velocity[index];
    }
}

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

bool ForceSolver::buildOmniDistribution(
    float wheel_radius,
    float wheel_center_distance,
    float* matrix
) {
    if (matrix == nullptr || wheel_center_distance == 0.0f || wheel_radius == 0.0f) {
        return false;
    }
    constexpr float WHEEL_X[4] = { 0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f };
    constexpr float WHEEL_Y[4] = { -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f };
    const float translation_scale = wheel_radius * 0.5f;
    const float rotation_scale = -wheel_radius / (4.0f * wheel_center_distance);
    for (uint8_t index = 0; index < 4; ++index) {
        matrix[index * 3] = WHEEL_X[index] * translation_scale;
        matrix[index * 3 + 1] = WHEEL_Y[index] * translation_scale;
        matrix[index * 3 + 2] = rotation_scale;
    }
    return true;
}
