/**
 * @file alg_slip_compensation2.cpp
 * @brief 基于执行器零空间的轮间打滑负载重分配实现。
 */

#include "alg_slip_compensation2.hpp"

#include <algorithm>
#include <cmath>

namespace slip_redistribution {

namespace {

constexpr float kTiny = 1.0e-6f;
constexpr float kInvalidRatio = 0.0f;

float constrainValue(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

float moveToward(float value, float target, float maximum_delta)
{
    if (target > value + maximum_delta) {
        return value + maximum_delta;
    }
    if (target < value - maximum_delta) {
        return value - maximum_delta;
    }
    return target;
}

int8_t signOf(float value)
{
    return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

bool configValid(const Config& config)
{
    return std::isfinite(config.dt_s)
        && config.dt_s > 0.0f
        && std::isfinite(config.min_tracking_speed_rad_s)
        && config.min_tracking_speed_rad_s >= 0.0f
        && std::isfinite(config.speed_ratio_filter_alpha)
        && config.speed_ratio_filter_alpha >= 0.0f
        && config.speed_ratio_filter_alpha < 1.0f
        && std::isfinite(config.normal_ratio_enter)
        && std::isfinite(config.normal_ratio_exit)
        && config.normal_ratio_enter > 1.0f
        && config.normal_ratio_exit >= 1.0f
        && config.normal_ratio_exit < config.normal_ratio_enter
        && std::isfinite(config.normal_ratio_spread)
        && config.normal_ratio_spread >= 0.0f
        && std::isfinite(config.abnormal_ratio_enter)
        && std::isfinite(config.abnormal_ratio_exit)
        && config.abnormal_ratio_enter > config.normal_ratio_enter
        && config.abnormal_ratio_exit >= config.normal_ratio_exit
        && config.abnormal_ratio_exit < config.abnormal_ratio_enter
        && std::isfinite(config.abnormal_ratio_spread)
        && config.abnormal_ratio_spread >= config.normal_ratio_spread
        && std::isfinite(config.load_confidence_min)
        && config.load_confidence_min >= 0.0f
        && config.load_confidence_min <= 1.0f
        && std::isfinite(config.max_adjustment_fraction)
        && config.max_adjustment_fraction > 0.0f
        && config.max_adjustment_fraction <= 1.0f
        && std::isfinite(config.adjustment_rate_per_s)
        && config.adjustment_rate_per_s > 0.0f
        && std::isfinite(config.recovery_rate_per_s)
        && config.recovery_rate_per_s > 0.0f;
}

} // namespace

Config defaultConfig()
{
    Config config {};
    config.enabled = true;
    config.dt_s = 0.001f;
    config.min_tracking_speed_rad_s = 4.0f;
    config.speed_ratio_filter_alpha = 0.75f;
    config.normal_ratio_enter = 1.20f;
    config.normal_ratio_exit = 1.08f;
    config.normal_ratio_spread = 0.18f;
    config.abnormal_ratio_enter = 1.75f;
    config.abnormal_ratio_exit = 1.40f;
    config.abnormal_ratio_spread = 0.45f;
    config.load_confidence_min = 0.25f;
    config.max_adjustment_fraction = 0.35f;
    config.adjustment_rate_per_s = 3.0f;
    config.recovery_rate_per_s = 6.0f;
    return config;
}

bool SlipRedistributor::init(
    const Config& config,
    const float* nullspace,
    uint8_t actuator_count
)
{
    if (!configValid(config) || nullspace == nullptr
        || actuator_count < 2U
        || actuator_count > ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX) {
        this->config_.enabled = false;
        this->actuator_count_ = 0U;
        this->status_ = Status::InvalidInput;
        this->reset();
        return false;
    }

    float norm_squared = 0.0f;
    for (uint8_t index = 0U; index < actuator_count; ++index) {
        if (!std::isfinite(nullspace[index])) {
            this->config_.enabled = false;
            this->actuator_count_ = 0U;
            this->status_ = Status::InvalidInput;
            return false;
        }
        norm_squared += nullspace[index] * nullspace[index];
    }
    if (!std::isfinite(norm_squared) || norm_squared <= kTiny) {
        this->config_.enabled = false;
        this->actuator_count_ = 0U;
        this->status_ = Status::InvalidInput;
        return false;
    }

    const float norm = std::sqrt(norm_squared);
    this->config_ = config;
    this->actuator_count_ = actuator_count;
    for (uint8_t index = 0U; index < actuator_count; ++index) {
        this->nullspace_[index] = nullspace[index] / norm;
    }
    this->reset();
    this->status_ = config.enabled ? Status::Ready : Status::Disabled;
    return true;
}

void SlipRedistributor::resetWheelState()
{
    std::fill_n(
        this->speed_ratio_,
        ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX,
        0.0f
    );
    std::fill_n(
        this->ratio_ready_,
        ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX,
        false
    );
    this->loaded_wheel_mask_ = 0U;
    this->abnormal_wheel_mask_ = 0U;
    this->limited_wheel_mask_ = 0U;
}

void SlipRedistributor::reset()
{
    this->coefficient_ = 0.0f;
    this->resetWheelState();
    this->normal_active_ = false;
    this->abnormal_active_ = false;
    this->status_ = this->config_.enabled ? Status::Ready : Status::Disabled;
}

bool SlipRedistributor::feasibleRange(
    const float* base_command,
    float command_limit,
    float* lower,
    float* upper
) const
{
    if (base_command == nullptr || lower == nullptr || upper == nullptr
        || !std::isfinite(command_limit) || command_limit <= 0.0f) {
        return false;
    }

    float base_norm_squared = 0.0f;
    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        if (!std::isfinite(base_command[index])) {
            return false;
        }
        base_norm_squared += base_command[index] * base_command[index];
    }
    const float base_norm = std::sqrt(base_norm_squared);
    if (!std::isfinite(base_norm) || base_norm <= kTiny) {
        return false;
    }

    float range_lower = -this->config_.max_adjustment_fraction;
    float range_upper = this->config_.max_adjustment_fraction;
    const float sign_threshold = command_limit * 0.02f;
    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        const float direction = base_norm * this->nullspace_[index];
        if (std::fabs(direction) <= kTiny) {
            continue;
        }

        float output_lower = -command_limit;
        float output_upper = command_limit;
        if (base_command[index] > sign_threshold) {
            output_lower = 0.0f;
        } else if (base_command[index] < -sign_threshold) {
            output_upper = 0.0f;
        }

        const float bound_a = (output_lower - base_command[index]) / direction;
        const float bound_b = (output_upper - base_command[index]) / direction;
        if (!std::isfinite(bound_a) || !std::isfinite(bound_b)) {
            return false;
        }
        range_lower = std::fmax(range_lower, std::fmin(bound_a, bound_b));
        range_upper = std::fmin(range_upper, std::fmax(bound_a, bound_b));
    }

    if (range_lower > range_upper) {
        return false;
    }
    *lower = range_lower;
    *upper = range_upper;
    return true;
}

bool SlipRedistributor::update(
    const float* base_command,
    const float* actual_speed,
    const float* target_speed,
    const float* confidence,
    uint8_t slip_wheel_mask,
    bool slip_detection_valid,
    float command_limit,
    float* output_command
)
{
    if (base_command == nullptr || actual_speed == nullptr
        || target_speed == nullptr || output_command == nullptr
        || !std::isfinite(command_limit) || command_limit <= 0.0f
        || this->actuator_count_ == 0U) {
        if (output_command != nullptr) {
            std::fill_n(
                output_command,
                this->actuator_count_,
                0.0f
            );
        }
        this->status_ = Status::InvalidInput;
        return false;
    }

    if (!this->config_.enabled) {
        std::copy_n(base_command, this->actuator_count_, output_command);
        this->status_ = Status::Disabled;
        return true;
    }

    const uint8_t previous_abnormal_mask = this->abnormal_wheel_mask_;
    this->resetWheelState();

    bool has_ratio = false;
    float minimum_ratio = 0.0f;
    float maximum_ratio = 0.0f;
    uint8_t minimum_index = 0U;
    uint8_t maximum_index = 0U;
    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        if (!std::isfinite(base_command[index])
            || !std::isfinite(actual_speed[index])
            || !std::isfinite(target_speed[index])
            || std::fabs(base_command[index]) > command_limit + kTiny) {
            std::fill_n(
                output_command,
                this->actuator_count_,
                0.0f
            );
            this->status_ = Status::InvalidInput;
            return false;
        }

        if (std::fabs(target_speed[index])
            < this->config_.min_tracking_speed_rad_s) {
            this->ratio_ready_[index] = false;
            this->speed_ratio_[index] = kInvalidRatio;
            continue;
        }

        const float raw_ratio = std::fabs(actual_speed[index])
            / std::fabs(target_speed[index]);
        if (!std::isfinite(raw_ratio)) {
            std::fill_n(
                output_command,
                this->actuator_count_,
                0.0f
            );
            this->status_ = Status::InvalidInput;
            return false;
        }

        const float alpha = this->config_.speed_ratio_filter_alpha;
        this->speed_ratio_[index] = this->ratio_ready_[index]
            ? alpha * this->speed_ratio_[index] + (1.0f - alpha) * raw_ratio
            : raw_ratio;
        this->ratio_ready_[index] = true;
        if (!has_ratio) {
            minimum_ratio = this->speed_ratio_[index];
            maximum_ratio = this->speed_ratio_[index];
            minimum_index = index;
            maximum_index = index;
            has_ratio = true;
        } else {
            if (this->speed_ratio_[index] < minimum_ratio) {
                minimum_ratio = this->speed_ratio_[index];
                minimum_index = index;
            }
            if (this->speed_ratio_[index] > maximum_ratio) {
                maximum_ratio = this->speed_ratio_[index];
                maximum_index = index;
            }
        }
    }

    float base_norm_squared = 0.0f;
    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        base_norm_squared += base_command[index] * base_command[index];
    }
    const float base_norm = std::sqrt(base_norm_squared);
    if (!std::isfinite(base_norm) || base_norm <= kTiny) {
        std::copy_n(base_command, this->actuator_count_, output_command);
        this->coefficient_ = 0.0f;
        this->status_ = Status::Ready;
        return true;
    }

    float lower = 0.0f;
    float upper = 0.0f;
    if (!this->feasibleRange(base_command, command_limit, &lower, &upper)) {
        std::fill_n(
            output_command,
            this->actuator_count_,
            0.0f
        );
        this->status_ = Status::InvalidInput;
        return false;
    }

    if (!has_ratio) {
        this->coefficient_ = moveToward(
            this->coefficient_,
            0.0f,
            this->config_.recovery_rate_per_s * this->config_.dt_s
        );
        this->coefficient_ = constrainValue(this->coefficient_, lower, upper);
        for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
            output_command[index] = base_command[index]
                + this->coefficient_ * base_norm * this->nullspace_[index];
        }
        this->status_ = std::fabs(this->coefficient_) > kTiny
            ? Status::Recovering
            : Status::Ready;
        return true;
    }

    const float ratio_spread = maximum_ratio - minimum_ratio;
    bool abnormal_active = false;
    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        if (!this->ratio_ready_[index]) {
            continue;
        }
        const uint8_t wheel_bit = static_cast<uint8_t>(1U << index);
        const bool enter_abnormal =
            this->speed_ratio_[index] >= this->config_.abnormal_ratio_enter
            && ratio_spread >= this->config_.abnormal_ratio_spread;
        const bool hold_abnormal =
            (previous_abnormal_mask & wheel_bit) != 0U
            && this->speed_ratio_[index] >= this->config_.abnormal_ratio_exit
            && ratio_spread >= this->config_.abnormal_ratio_spread * 0.5f;
        const bool abnormal = enter_abnormal || hold_abnormal;
        if (abnormal) {
            this->abnormal_wheel_mask_ |= wheel_bit;
            abnormal_active = true;
        }
    }
    this->abnormal_active_ = abnormal_active;

    uint8_t target_index = 0U;
    float desired_coefficient = 0.0f;
    if (abnormal_active) {
        this->normal_active_ = false;
        target_index = maximum_index;
    } else {
        const bool enter_normal =
            maximum_ratio >= this->config_.normal_ratio_enter
            && ratio_spread >= this->config_.normal_ratio_spread;
        const bool hold_normal =
            this->normal_active_
            && maximum_ratio >= this->config_.normal_ratio_exit
            && ratio_spread >= this->config_.normal_ratio_spread;
        this->normal_active_ = enter_normal || hold_normal;
        if (this->normal_active_) {
            bool reliable_found = false;
            float reliable_minimum = 0.0f;
            for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
                if (!this->ratio_ready_[index]) {
                    continue;
                }
                const uint8_t wheel_bit = static_cast<uint8_t>(1U << index);
                const bool reported_slipping =
                    slip_detection_valid && (slip_wheel_mask & wheel_bit) != 0U;
                const bool reliable = !slip_detection_valid || confidence == nullptr
                    || confidence[index] >= this->config_.load_confidence_min;
                if (reliable
                    && !reported_slipping
                    && (!reliable_found
                        || this->speed_ratio_[index] < reliable_minimum)) {
                    reliable_found = true;
                    reliable_minimum = this->speed_ratio_[index];
                    target_index = index;
                }
            }
            if (!reliable_found) {
                target_index = minimum_index;
            }
            this->loaded_wheel_mask_ |=
                static_cast<uint8_t>(1U << target_index);
        }
    }

    if (abnormal_active) {
        const float direction = base_norm * this->nullspace_[target_index];
        if (std::fabs(direction) > kTiny) {
            // 沿零空间把异常高速轮命令向零移动。
            desired_coefficient =
                -base_command[target_index] / direction;
            desired_coefficient = constrainValue(
                desired_coefficient,
                lower,
                upper
            );
        }
    } else if (this->normal_active_) {
        const float direction = base_norm * this->nullspace_[target_index];
        const int8_t direction_sign = signOf(base_command[target_index] * direction);
        // 选中的相对低速轮沿原输出方向增载。
        desired_coefficient =
            static_cast<float>(direction_sign)
            * this->config_.max_adjustment_fraction;
        desired_coefficient = constrainValue(
            desired_coefficient,
            lower,
            upper
        );
    }

    const float maximum_delta =
        (abnormal_active || this->normal_active_
            ? this->config_.adjustment_rate_per_s
            : this->config_.recovery_rate_per_s)
        * this->config_.dt_s;
    this->coefficient_ = moveToward(
        this->coefficient_,
        desired_coefficient,
        maximum_delta
    );
    this->coefficient_ = constrainValue(this->coefficient_, lower, upper);

    for (uint8_t index = 0U; index < this->actuator_count_; ++index) {
        output_command[index] = base_command[index]
            + this->coefficient_ * base_norm * this->nullspace_[index];
        output_command[index] = constrainValue(
            output_command[index],
            -command_limit,
            command_limit
        );
        if (std::fabs(output_command[index]) >= command_limit - kTiny) {
            this->limited_wheel_mask_ |=
                static_cast<uint8_t>(1U << index);
        }
    }

    if (abnormal_active) {
        this->status_ = std::fabs(this->coefficient_ - desired_coefficient) > kTiny
            ? Status::Limited
            : Status::AbnormalSlip;
    } else if (this->normal_active_) {
        this->status_ = Status::NormalSlip;
    } else if (std::fabs(this->coefficient_) > kTiny) {
        this->status_ = Status::Recovering;
    } else {
        this->status_ = Status::Ready;
    }
    return true;
}

} // namespace slip_redistribution
