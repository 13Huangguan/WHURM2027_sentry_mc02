#ifndef ALG_KALMAN_FILTER_HPP
#define ALG_KALMAN_FILTER_HPP

#include <cstdint>
#include <cstdlib>
#include <memory>

#include "mat_matrix.hpp"

struct ChiSquareTest {
    bool test_flag = false;
    matrix::View chi_square_matrix {};
    float chi_square_data[1] {};
    float chi_square_test_thresholds = 0.0f;
    uint8_t chi_square_cnt = 0;
    bool result = false;
};

struct FreeKalmanBuffer {
    void operator()(float* buffer) const {
        std::free(buffer);
    }
};

// Runtime dimensions share one allocation at initialization, none during updates.
// Copy/move is disabled because matrix views borrow this object's storage.
struct KalmanFilter {
    KalmanFilter() = default;
    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;
    KalmanFilter(KalmanFilter&&) = delete;
    KalmanFilter& operator=(KalmanFilter&&) = delete;

    uint16_t sizeof_float = sizeof(float);
    uint8_t xhat_size = 0;
    uint8_t u_size = 0;
    uint8_t z_size = 0;
    float dt = 0.0f;
    float gain_scale = 1.0f;
    float* measured_vector = nullptr;
    float* control_vector = nullptr;
    ChiSquareTest chi_square_test {};

    struct Matrices {
        matrix::View xhat, xhatminus, u, z, b, a, at, h, ht, p, pminus, q, r, k;
        matrix::View k_denominator;
        matrix::View cache_matrix[2];
        matrix::View cache_vector[2];
    } mat {};

    arm_status mat_status = ARM_MATH_LENGTH_ERROR;

    // Raw data aliases are provided for the EKF Jacobians and configuration.
    struct Data {
        float *xhat, *xhatminus, *u, *z, *b, *a, *at, *h, *ht, *p, *pminus, *q, *r, *k;
        float* k_denominator;
        float* cache_matrix[2];
        float* cache_vector[2];
    } data {};

    bool skip_step1 = false;
    bool skip_step2 = false;
    bool skip_step3 = false;
    bool skip_step4 = false;
    bool skip_step5 = false;
    using Callback = void (*)(KalmanFilter*);
    Callback user_function0 = nullptr;
    Callback user_function1 = nullptr;
    Callback user_function2 = nullptr;
    Callback user_function3 = nullptr;
    Callback user_function4 = nullptr;
    Callback user_function5 = nullptr;
    Callback user_function6 = nullptr;
    float* output = nullptr;
    using Buffer = std::unique_ptr<float[], FreeKalmanBuffer>;
    Buffer storage;
    bool initialized = false;
};

bool kalmanFilterInit(KalmanFilter* kf, uint8_t xhat_size, uint8_t u_size, uint8_t z_size);
float* kalmanFilterUpdate(KalmanFilter* kf);

// Shared by linear Kalman and nonlinear EKF updates. Preserve the inverse in cache_matrix[1].
bool kalmanFilterInnovation(KalmanFilter* kf);
bool kalmanFilterGain(KalmanFilter* kf);

#endif // ALG_KALMAN_FILTER_HPP
