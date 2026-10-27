/**
 * @file alg_motion_compensation.cpp
 * @brief 四轮全向底盘运动估计与目标速度补偿实现。
 * @copyright WHU-LuojiaFox (c) 2026
 */

#include "alg_motion_compensation.hpp"

#include <cmath>

namespace motion_compensation {

namespace {

constexpr float kSqrtHalf = 0.70710678118654752440f;

/**
 * @brief 四轮在底盘坐标系 x 方向的基础安装系数。
 *
 * 轮序与文档及现有 app_chassis 保持一致。Yaw 旋转后还会叠加相应的
 * y 方向基础系数。
 */
constexpr float kWheelBaseX[kWheelCount] = {
    kSqrtHalf, kSqrtHalf, -kSqrtHalf, -kSqrtHalf
};

/**
 * @brief 四轮在底盘坐标系 y 方向的基础安装系数。
 */
constexpr float kWheelBaseY[kWheelCount] = {
    -kSqrtHalf, kSqrtHalf, kSqrtHalf, -kSqrtHalf
};

/**
 * @brief 求解 3x3 线性方程 Ax=b。
 * @note 使用带主元选择的双精度高斯消元，避免直接写死四组逆解公式。
 */
bool solve3x3(double matrix[3][4], double output[3]) {
    for (uint8_t column = 0U; column < 3U; ++column) {
        uint8_t pivot = column;
        for (uint8_t row = static_cast<uint8_t>(column + 1U); row < 3U; ++row) {
            if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::fabs(matrix[pivot][column]) < 1.0e-9) {
            return false;
        }
        if (pivot != column) {
            for (uint8_t item = column; item < 4U; ++item) {
                const double temporary = matrix[column][item];
                matrix[column][item] = matrix[pivot][item];
                matrix[pivot][item] = temporary;
            }
        }

        const double divisor = matrix[column][column];
        for (uint8_t item = column; item < 4U; ++item) {
            matrix[column][item] /= divisor;
        }

        for (uint8_t row = 0U; row < 3U; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = matrix[row][column];
            for (uint8_t item = column; item < 4U; ++item) {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }

    for (uint8_t row = 0U; row < 3U; ++row) {
        output[row] = matrix[row][3];
    }
    return true;
}

float clampCorrection(float value, float maximum) {
    if (!std::isfinite(value) || maximum <= 0.0f) {
        return value;
    }
    if (value > maximum) {
        return maximum;
    }
    if (value < -maximum) {
        return -maximum;
    }
    return value;
}

} // namespace

Config defaultConfig() {
    Config config {};
    config.acceleration_process_noise = 0.05f;
    config.wheel_measurement_variance = 0.0025f;
    config.acceleration_measurement_variance = 0.25f;
    config.initial_velocity_variance = 1.0f;
    config.initial_acceleration_variance = 4.0f;
    config.correction_gain_per_rad_s = 0.05f;
    config.max_correction_mps = 0.25f;
    return config;
}

bool OmniMotionCompensator::configValid(const Config& config) {
    return std::isfinite(config.acceleration_process_noise)
        && config.acceleration_process_noise > 0.0f
        && std::isfinite(config.wheel_measurement_variance)
        && config.wheel_measurement_variance > 0.0f
        && std::isfinite(config.acceleration_measurement_variance)
        && config.acceleration_measurement_variance > 0.0f
        && std::isfinite(config.initial_velocity_variance)
        && config.initial_velocity_variance >= 0.0f
        && std::isfinite(config.initial_acceleration_variance)
        && config.initial_acceleration_variance >= 0.0f
        && std::isfinite(config.correction_gain_per_rad_s)
        && config.correction_gain_per_rad_s >= 0.0f
        && std::isfinite(config.max_correction_mps)
        && config.max_correction_mps >= 0.0f;
}

bool OmniMotionCompensator::init() {
    return init(defaultConfig());
}

bool OmniMotionCompensator::init(const Config& config) {
    if (!configValid(config)) {
        configured_ = false;
        initialized_ = false;
        return false;
    }

    config_ = config;
    configured_ = true;
    reset();
    return true;
}

void OmniMotionCompensator::reset() {
    vx_axis_ = {};
    vy_axis_ = {};
    yaw_rate_rad_s_ = 0.0f;
    valid_wheel_mask_ = 0U;
    initialized_ = false;
    velocity_measurement_valid_ = false;
}

void OmniMotionCompensator::resetAxis(
    AxisFilter* axis,
    float velocity,
    float acceleration,
    const Config& config
) {
    axis->velocity = velocity;
    axis->acceleration = acceleration;
    axis->p00 = config.initial_velocity_variance;
    axis->p01 = 0.0f;
    axis->p11 = config.initial_acceleration_variance;
}

void OmniMotionCompensator::predictAxis(
    AxisFilter* axis,
    float dt_s,
    const Config& config
) {
    const float dt2 = dt_s * dt_s;
    const float dt3 = dt2 * dt_s;
    const float dt4 = dt2 * dt2;
    const float q = config.acceleration_process_noise;

    // 状态方程：v(k+1)=v(k)+a(k)dt，a(k+1)=a(k)。
    axis->velocity += axis->acceleration * dt_s;

    // P(k+1)=F*P*F^T+Q，Q 由连续加速度过程噪声离散化。
    const float old_p00 = axis->p00;
    const float old_p01 = axis->p01;
    const float old_p11 = axis->p11;
    axis->p00 = old_p00 + 2.0f * dt_s * old_p01 + dt2 * old_p11
              + q * dt4 * 0.25f;
    axis->p01 = old_p01 + dt_s * old_p11 + q * dt3 * 0.5f;
    axis->p11 = old_p11 + q * dt2;
}

void OmniMotionCompensator::updateVelocity(
    AxisFilter* axis,
    float measured_velocity,
    float variance
) {
    // H=[1,0]，标量测量更新。
    const float innovation_variance = axis->p00 + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0f) {
        return;
    }

    const float gain_v = axis->p00 / innovation_variance;
    const float gain_a = axis->p01 / innovation_variance;
    const float innovation = measured_velocity - axis->velocity;
    const float old_p01 = axis->p01;

    axis->velocity += gain_v * innovation;
    axis->acceleration += gain_a * innovation;
    axis->p00 *= 1.0f - gain_v;
    axis->p01 *= 1.0f - gain_v;
    axis->p11 -= gain_a * old_p01;

    // 数值误差不应产生负方差或破坏对称性。
    axis->p00 = axis->p00 > 0.0f ? axis->p00 : 0.0f;
    axis->p11 = axis->p11 > 0.0f ? axis->p11 : 0.0f;
}

void OmniMotionCompensator::updateAcceleration(
    AxisFilter* axis,
    float measured_acceleration,
    float variance
) {
    // H=[0,1]，标量测量更新。
    const float innovation_variance = axis->p11 + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0f) {
        return;
    }

    const float gain_v = axis->p01 / innovation_variance;
    const float gain_a = axis->p11 / innovation_variance;
    const float innovation = measured_acceleration - axis->acceleration;
    const float old_p01 = axis->p01;

    axis->velocity += gain_v * innovation;
    axis->acceleration += gain_a * innovation;
    axis->p00 -= gain_v * old_p01;
    axis->p01 -= gain_a * old_p01;
    axis->p11 *= 1.0f - gain_a;

    axis->p00 = axis->p00 > 0.0f ? axis->p00 : 0.0f;
    axis->p11 = axis->p11 > 0.0f ? axis->p11 : 0.0f;
}

bool OmniMotionCompensator::solveVelocity(
    const float wheel_linear_velocity_mps[kWheelCount],
    uint8_t valid_mask,
    float yaw_rad,
    float* vx_mps,
    float* vy_mps
) {
    if (wheel_linear_velocity_mps == nullptr || vx_mps == nullptr || vy_mps == nullptr
        || !std::isfinite(yaw_rad)) {
        return false;
    }

    const float sin_yaw = std::sin(yaw_rad);
    const float cos_yaw = std::cos(yaw_rad);
    double ata[3][3] = {};
    double atb[3] = {};
    uint8_t valid_count = 0U;

    for (uint8_t wheel = 0U; wheel < kWheelCount; ++wheel) {
        if ((valid_mask & static_cast<uint8_t>(1U << wheel)) == 0U) {
            continue;
        }
        if (!std::isfinite(wheel_linear_velocity_mps[wheel])) {
            return false;
        }

        // 轮速方程：
        // v_wheel = row_x*vx + row_y*vy - wR。
        // 将 wR 作为第三个未知量，可由三到四个轮速联合求解。
        const float row[3] = {
            cos_yaw * kWheelBaseX[wheel] - sin_yaw * kWheelBaseY[wheel],
            sin_yaw * kWheelBaseX[wheel] + cos_yaw * kWheelBaseY[wheel],
            -1.0f
        };
        for (uint8_t i = 0U; i < 3U; ++i) {
            atb[i] += static_cast<double>(row[i])
                    * static_cast<double>(wheel_linear_velocity_mps[wheel]);
            for (uint8_t j = 0U; j < 3U; ++j) {
                ata[i][j] += static_cast<double>(row[i])
                           * static_cast<double>(row[j]);
            }
        }
        ++valid_count;
    }

    if (valid_count < 3U) {
        return false;
    }

    double augmented[3][4] = {};
    for (uint8_t row = 0U; row < 3U; ++row) {
        for (uint8_t column = 0U; column < 3U; ++column) {
            augmented[row][column] = ata[row][column];
        }
        augmented[row][3] = atb[row];
    }

    double solution[3] = {};
    if (!solve3x3(augmented, solution)
        || !std::isfinite(solution[0])
        || !std::isfinite(solution[1])) {
        return false;
    }

    *vx_mps = static_cast<float>(solution[0]);
    *vy_mps = static_cast<float>(solution[1]);
    return true;
}

bool OmniMotionCompensator::update(const Measurement& measurement) {
    if (!configured_ || !std::isfinite(measurement.dt_s) || measurement.dt_s <= 0.0f) {
        return false;
    }

    float measured_vx = 0.0f;
    float measured_vy = 0.0f;
    const bool wheel_measurement_valid = solveVelocity(
        measurement.wheel_linear_velocity_mps,
        measurement.valid_wheel_mask,
        measurement.yaw_rad,
        &measured_vx,
        &measured_vy
    );
    const bool acceleration_valid = measurement.acceleration_valid
        && std::isfinite(measurement.acceleration_x_mps2)
        && std::isfinite(measurement.acceleration_y_mps2);

    if (!wheel_measurement_valid && !acceleration_valid) {
        velocity_measurement_valid_ = false;
        valid_wheel_mask_ = measurement.valid_wheel_mask;
        return false;
    }

    if (!initialized_) {
        resetAxis(
            &vx_axis_,
            wheel_measurement_valid ? measured_vx : 0.0f,
            acceleration_valid ? measurement.acceleration_x_mps2 : 0.0f,
            config_
        );
        resetAxis(
            &vy_axis_,
            wheel_measurement_valid ? measured_vy : 0.0f,
            acceleration_valid ? measurement.acceleration_y_mps2 : 0.0f,
            config_
        );
        initialized_ = true;
    } else {
        predictAxis(&vx_axis_, measurement.dt_s, config_);
        predictAxis(&vy_axis_, measurement.dt_s, config_);

        if (acceleration_valid) {
            updateAcceleration(
                &vx_axis_,
                measurement.acceleration_x_mps2,
                config_.acceleration_measurement_variance
            );
            updateAcceleration(
                &vy_axis_,
                measurement.acceleration_y_mps2,
                config_.acceleration_measurement_variance
            );
        }

        if (wheel_measurement_valid) {
            updateVelocity(
                &vx_axis_,
                measured_vx,
                config_.wheel_measurement_variance
            );
            updateVelocity(
                &vy_axis_,
                measured_vy,
                config_.wheel_measurement_variance
            );
        }
    }

    yaw_rate_rad_s_ = std::isfinite(measurement.yaw_rate_rad_s)
        ? measurement.yaw_rate_rad_s : 0.0f;
    valid_wheel_mask_ = measurement.valid_wheel_mask;
    velocity_measurement_valid_ = wheel_measurement_valid;
    return true;
}

Estimate OmniMotionCompensator::estimate() const {
    Estimate result {};
    result.vx_mps = vx_axis_.velocity;
    result.vy_mps = vy_axis_.velocity;
    result.ax_mps2 = vx_axis_.acceleration;
    result.ay_mps2 = vy_axis_.acceleration;
    result.yaw_rate_rad_s = yaw_rate_rad_s_;
    result.initialized = initialized_;
    result.velocity_measurement_valid = velocity_measurement_valid_;
    result.valid_wheel_mask = valid_wheel_mask_;
    return result;
}

CorrectedVelocity OmniMotionCompensator::compensate(
    float target_vx_mps,
    float target_vy_mps
) const {
    CorrectedVelocity result {};
    result.vx_mps = target_vx_mps;
    result.vy_mps = target_vy_mps;

    if (!initialized_
        || !std::isfinite(target_vx_mps)
        || !std::isfinite(target_vy_mps)
        || !std::isfinite(yaw_rate_rad_s_)) {
        return result;
    }

    // 文档中的 P 补偿：Kp = K*|w|，作用于目标与估计值之差。
    const float gain = config_.correction_gain_per_rad_s * std::fabs(yaw_rate_rad_s_);
    float correction_x = gain * (target_vx_mps - vx_axis_.velocity);
    float correction_y = gain * (target_vy_mps - vy_axis_.velocity);

    // 限制补偿向量模长，避免滤波瞬态或错误轮速造成过大的目标跳变。
    const float correction_norm = std::sqrt(
        correction_x * correction_x + correction_y * correction_y
    );
    if (config_.max_correction_mps > 0.0f
        && correction_norm > config_.max_correction_mps) {
        const float scale = config_.max_correction_mps / correction_norm;
        correction_x *= scale;
        correction_y *= scale;
    }

    result.correction_x_mps = clampCorrection(
        correction_x,
        config_.max_correction_mps
    );
    result.correction_y_mps = clampCorrection(
        correction_y,
        config_.max_correction_mps
    );
    result.vx_mps = target_vx_mps + result.correction_x_mps;
    result.vy_mps = target_vy_mps + result.correction_y_mps;
    result.applied = result.correction_x_mps != 0.0f
        || result.correction_y_mps != 0.0f;
    return result;
}

bool OmniMotionCompensator::isConfigured() const {
    return configured_;
}

bool OmniMotionCompensator::isInitialized() const {
    return initialized_;
}

Config OmniMotionCompensator::config() const {
    return config_;
}

} // namespace motion_compensation

