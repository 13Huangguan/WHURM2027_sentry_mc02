#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <type_traits>

#include "alg_basic.hpp"
#include "alg_force.hpp"
#include "alg_kalman_filter.hpp"
#include "alg_lpf.hpp"
#include "alg_quaternion.hpp"
#include "mat_matrix.hpp"

namespace {
int checks = 0;
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        std::exit(1);
    }
}
void near(float actual, float expected, float tolerance, const char* label) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr, "%s: actual=%g expected=%g\n", label, actual, expected);
        check(false, label);
    }
    ++checks;
}

void testMatrix() {
    for (uint16_t rows = 1; rows <= 8; ++rows) {
        for (uint16_t cols = 1; cols <= 7; ++cols) {
            float values[56], vector[7], output[10];
            std::fill_n(output, 10, 12345.0f);
            for (int index = 0; index < rows * cols; ++index) {
                values[index] = std::sin(index * 0.13f);
            }
            for (int col = 0; col < cols; ++col) {
                vector[col] = col * 0.37f - 1.0f;
            }
            check(
                matrix::multiplyVector(rows, cols, values, vector, output + 1) == ARM_MATH_SUCCESS,
                "specialized/generic vector product status"
            );
            for (int row = 0; row < rows; ++row) {
                float expected = 0.0f;
                for (int col = 0; col < cols; ++col) {
                    expected += values[row * cols + col] * vector[col];
                }
                near(output[row + 1], expected, 2e-6f, "specialized/generic vector reference");
            }
            check(output[0] == 12345.0f && output[rows + 1] == 12345.0f, "vector output bounds");
        }
    }
    float scalar = 1.0f, result = 0.0f;
    check(
        matrix::multiplyVector(0, 1, &scalar, &scalar, &result) == ARM_MATH_LENGTH_ERROR,
        "vector zero dimension rejected"
    );
    check(
        matrix::multiplyVector(1, 1, nullptr, &scalar, &result) == ARM_MATH_ARGUMENT_ERROR,
        "vector null buffer rejected"
    );
    check(
        matrix::multiplyVector(1, 1, &scalar, &scalar, &scalar) == ARM_MATH_SIZE_MISMATCH,
        "vector alias rejected"
    );
    const float values[6] = { 1, 2, 3, 4, 5, 6 };
    matrix::Matrix<2, 3> value(values);
    const auto product = value * value.trans();
    near(product[0][0], 14, 1e-6f, "rectangular multiplication");
    near(product[0][1], 32, 1e-6f, "rectangular multiplication");
    near(product[1][1], 77, 1e-6f, "rectangular multiplication");
    near(value.norm(), std::sqrt(91.0f), 1e-6f, "Frobenius norm");
    auto copy(value);
    copy[0][0] = 10;
    check(copy.arm_mat.pData == copy.data(), "copy descriptor owns its own storage");
    const auto scaled = (value + value) * 0.5f - value;
    near(scaled.norm(), 0.0f, 0.0f, "fully overwritten result storage");
    near(value.template block<1, 2>(-1, 0).norm(), 0.0f, 0.0f, "invalid block stays zero");
    near(value[0][0], 1, 0, "copy independence");
    copy = value;
    copy = copy;
    near(copy.template block<1, 2>(1, 1)[0][1], 6, 0, "block copy");
    const float invertible[4] = { 0, 2, 1, 3 };
    const matrix::Matrix<2, 2> input(invertible);
    matrix::Matrix<2, 2> inverse;
    check(matrix::inverse(input, inverse) == ARM_MATH_SUCCESS, "inverse pivots rows");
    const auto identity = input * inverse;
    near(identity[0][0], 1, 1e-5f, "inverse identity");
    near(identity[1][0], 0, 1e-5f, "inverse identity");
    near(input[0][0], 0, 0, "inverse preserves input");
    matrix::Matrix<2, 2> singular;
    check(
        matrix::inverse(singular, inverse) == ARM_MATH_SINGULAR,
        "singular inverse reports failure"
    );
    auto lhs = matrix::eye<3, 3>();
    matrix::Matrix<3, 1> x, y;
    x[0][0] = 1;
    y[1][0] = 1;
    near(matrix::cross(x, y)[2][0], 1, 0, "cross product orientation");
    check(
        matrix::multiply(&lhs.arm_mat, &x.arm_mat, &x.arm_mat) == ARM_MATH_SIZE_MISMATCH,
        "in-place multiplication rejected"
    );
}

void testForce() {
    for (uint8_t actuators = 1; actuators <= 8; ++actuators) {
        for (uint8_t dofs = 1; dofs <= 4; ++dofs) {
            ForceActuatorParam params[8] {};
            float distribution[32] {};
            float previous_velocity[8] {}, filtered_velocity[8] {};
            for (uint8_t index = 0; index < actuators; ++index) {
                params[index] = { 0.7f,  19.0f,   int8_t(index % 2 ? -1 : 1),
                                  1.5f,  0.0001f, 0.1f,
                                  0.02f, 0.3f,    0.2f };
                for (uint8_t col = 0; col < dofs; ++col) {
                    distribution[index * dofs + col] = (int(index + col) - 3) * 0.1f;
                }
            }
            ForceSolver solver;
            solver.init(params, actuators, dofs, 0.001f);
            solver.setDistribution(distribution);
            solver.setEnable(true);
            for (int step = 0; step < 100; ++step) {
                float target[4] {}, position[8] {}, velocity[8] {}, feedforward[8] {};
                for (int col = 0; col < dofs; ++col) {
                    target[col] = std::sin(step * 0.1f + col) * 3.0f;
                }
                for (int index = 0; index < actuators; ++index) {
                    position[index] = std::sin(step * 0.03f + index);
                    velocity[index] = std::cos(step * 0.03f + index);
                    feedforward[index] = 0.03f * index;
                    if (step == 0) {
                        filtered_velocity[index] = velocity[index];
                    }
                    filtered_velocity[index] =
                        0.9f * filtered_velocity[index] + 0.1f * velocity[index];
                }
                solver.setTarget(target);
                solver.setNow(position, velocity);
                solver.setFeedforwardTorque(feedforward);
                solver.timCalculatePeriodElapsedCallback();
                for (int index = 0; index < actuators; ++index) {
                    float expected = 0;
                    for (int col = 0; col < dofs; ++col) {
                        expected += distribution[index * dofs + col] * target[col];
                    }
                    expected += params[index].inertia
                        * ((filtered_velocity[index] - previous_velocity[index]) / 0.001f);
                    expected += params[index].viscous_friction * filtered_velocity[index];
                    expected +=
                        params[index].coulomb_friction * basic_math::sign(filtered_velocity[index]);
                    expected += params[index].gravity_amplitude
                        * std::cos(position[index] - params[index].gravity_offset);
                    expected += feedforward[index];
                    expected = basic_math::constrain(
                        expected,
                        -params[index].torque_max,
                        params[index].torque_max
                    );
                    near(solver.getTorque(index), expected, 3e-5f, "force scalar reference");
                    near(
                        solver.getOut(index),
                        expected / (0.7f * 19.0f) * params[index].direction,
                        3e-6f,
                        "force conversion reference"
                    );
                    previous_velocity[index] = filtered_velocity[index];
                }
            }
            solver.setEnable(false);
            solver.timCalculatePeriodElapsedCallback();
            near(solver.getOut(0), 0, 0, "force disabled");
        }
    }
    ForceActuatorParam params[4] {};
    for (auto& param: params) {
        param.torque_constant = param.reduction_ratio = param.direction = 1;
        param.torque_max = 100;
    }
    ForceSolver solver;
    solver.init(params, 4, 3, 0.001f);
    float distribution[12];
    check(ForceSolver::buildOmniDistribution(0.05f, 0.25f, distribution), "omni matrix builds");
    solver.setDistribution(distribution);
    const float target[3] = { 10, -4, 1.5f };
    solver.setTarget(target);
    solver.setEnable(true);
    solver.timCalculatePeriodElapsedCallback();
    const float x[4] = { 0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f };
    const float y[4] = { -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f };
    float restored[3] {};
    for (int index = 0; index < 4; ++index) {
        restored[0] += x[index] * solver.getTorque(index) / 0.05f;
        restored[1] += y[index] * solver.getTorque(index) / 0.05f;
        restored[2] -= 0.25f * solver.getTorque(index) / 0.05f;
    }
    for (int index = 0; index < 3; ++index) {
        near(restored[index], target[index], 1e-4f, "omni wrench reconstruction");
    }
    solver.init(nullptr, 255, 255, 0);
    check(solver.getActuatorNum() == 0, "null force parameter buffer");
    params[0].torque_constant = 0;
    solver.init(params, 1, 1, 0.001f);
    solver.setTarget(target);
    solver.setEnable(true);
    solver.timCalculatePeriodElapsedCallback();
    near(solver.getOut(0), 0, 0, "zero torque constant");
}

void testKalman() {
    static_assert(
        !std::is_copy_constructible<KalmanFilter>::value,
        "borrowed views must not be copied"
    );
    KalmanFilter filter;
    check(!kalmanFilterInit(&filter, 0, 0, 1), "invalid Kalman dimension");
    check(kalmanFilterUpdate(&filter) == nullptr, "uninitialized Kalman");
    check(kalmanFilterInit(&filter, 1, 1, 1), "scalar Kalman initialization");
    filter.data.a[0] = filter.data.b[0] = filter.data.h[0] = filter.data.p[0] = 1;
    filter.data.q[0] = 0.1f;
    filter.data.r[0] = 0.5f;
    filter.control_vector[0] = 2;
    near(kalmanFilterUpdate(&filter)[0], 0.625f, 1e-5f, "controlled scalar Kalman posterior");
    near(filter.data.p[0], 0.34375f, 1e-5f, "controlled scalar covariance");
    check(kalmanFilterInit(&filter, 1, 0, 4), "measurement dimension larger than state");
    filter.data.a[0] = filter.data.p[0] = 1;
    for (int index = 0; index < 4; ++index) {
        filter.data.h[index] = 1;
        filter.data.r[index * 4 + index] = 1;
        filter.measured_vector[index] = 2;
    }
    near(kalmanFilterUpdate(&filter)[0], 1.6f, 1e-5f, "four measurements one state");
    near(filter.data.p[0], 0.2f, 1e-5f, "four measurement covariance");
    check(filter.mat_status == ARM_MATH_SUCCESS, "rectangular scratch dimensions");
    check(kalmanFilterInit(&filter, 1, 0, 1), "reinitialize Kalman");
    filter.data.a[0] = 1;
    kalmanFilterUpdate(&filter);
    check(filter.mat_status == ARM_MATH_SINGULAR, "singular innovation remains visible");
    near(filter.output[0], 0, 0, "failed update retains published output");
}

void testQuaternion() {
    float a[36] {}, p[36] {};
    for (int index = 0; index < 6; ++index) {
        a[index * 6 + index] = p[index * 6 + index] = 1;
    }
    QuaternionInfo info {};
    quaternionEkfInit(&info, 10, 0.001f, 100000, a, p);
    check(info.initialized, "EKF initializes");
    float gyro[3] {}, accel[3] = { 0, 0, 9.718f };
    for (int step = 0; step < 1000; ++step) {
        quaternionEkfUpdate(&info, gyro, accel, 0.001f);
        check(info.quaternion_ekf.mat_status == ARM_MATH_SUCCESS, "stationary EKF shapes");
    }
    near(info.quat[0], 1, 1e-6f, "stationary quaternion");
    check(
        info.quaternion_ekf.chi_square_test.test_flag,
        "zero gyro is stationary without reciprocal-of-zero"
    );
    near(info.euler_angle[0], 0, 1e-6f, "stationary yaw");
    gyro[2] = 0.2f;
    for (int step = 0; step < 5000; ++step) {
        quaternionEkfUpdate(&info, gyro, accel, 0.001f);
    }
    near(info.euler_angle[0], 1.0f, 1e-3f, "one radian yaw integration");
    float norm = 0;
    for (float value: info.quat) {
        norm += value * value;
    }
    near(norm, 1, 1e-5f, "quaternion remains normalized");
    const float previous_yaw = info.euler_angle[0];
    accel[2] = 0;
    quaternionEkfUpdate(&info, gyro, accel, 0.001f);
    near(info.euler_angle[0], previous_yaw, 0, "zero acceleration rejected");
    accel[2] = std::numeric_limits<float>::quiet_NaN();
    quaternionEkfUpdate(&info, gyro, accel, 0.001f);
    near(info.euler_angle[0], previous_yaw, 0, "NaN acceleration rejected");
    // With P=I and q=(1,0,0,0), H H^T=4I. Expected chi-square is ||z-h||^2/(4+R).
    quaternionEkfInit(&info, 0, 0, 2, a, p);
    gyro[2] = 0;
    accel[0] = 0.6f * 9.718f;
    accel[2] = 0.8f * 9.718f;
    quaternionEkfUpdate(&info, gyro, accel, 0.001f);
    near(
        info.quaternion_ekf.chi_square_test.chi_square_data[0],
        0.4f / 6.0f,
        1e-5f,
        "chi-square residual transpose regression"
    );
}

void testLpf() {
    LowPassFilter1p filter {};
    lowPassFilter1pInit(&filter, 0.9f);
    near(lowPassFilter1pUpdate(&filter, 2), 2, 1e-6f, "LPF first sample");
    lowPassFilter1pUpdate(&filter, 3);
    lowPassFilter1pInit(&filter, 0.9f);
    near(lowPassFilter1pUpdate(&filter, 8), 8, 1e-6f, "LPF reinitialization resets ready flag");
}
} // namespace

int main() {
    testMatrix();
    testForce();
    testKalman();
    testQuaternion();
    testLpf();
    std::printf("PASS: %d matrix/force/Kalman/EKF/LPF checks\n", checks);
}
