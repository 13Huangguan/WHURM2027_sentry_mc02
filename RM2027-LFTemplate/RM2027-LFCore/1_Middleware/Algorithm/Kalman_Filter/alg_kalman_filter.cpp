
#include "alg_kalman_filter.hpp"

#include <algorithm>
#include <cstring>

#include "prf_cycle_counter.hpp"

bool kalmanFilterInit(KalmanFilter* kf, uint8_t xhat_size, uint8_t u_size, uint8_t z_size) {
#if LFCORE_PROFILE
    profiling::initCycleCounter();
#endif
    if (kf == nullptr) {
        return false;
    }
    kf->initialized = false;
    kf->mat_status = ARM_MATH_LENGTH_ERROR;
    if (xhat_size == 0 || z_size == 0) {
        return false;
    }
    const size_t n = xhat_size;
    const size_t u = u_size;
    const size_t z = z_size;
    const size_t max_dim = std::max(n, z);
    const size_t count = 3 * n + 2 * z + 2 * u + n * u + 5 * n * n + 3 * n * z + 2 * z * z
        + 2 * max_dim * max_dim + 2 * max_dim;
    KalmanFilter::Buffer storage(static_cast<float*>(std::calloc(count, sizeof(float))));
    if (!storage) {
        return false;
    }
    kf->storage = std::move(storage);
    float* cursor = kf->storage.get();
    auto allocate = [&cursor](size_t size) {
        float* data = cursor;
        cursor += size;
        return data;
    };
    auto bind = [&allocate](matrix::View& view, float*& data, uint16_t rows, uint16_t cols) {
        data = allocate(static_cast<size_t>(rows) * cols);
        matrix::init(&view, rows, cols, data);
    };
    kf->xhat_size = xhat_size;
    kf->u_size = u_size;
    kf->z_size = z_size;
    kf->dt = 0.0f;
    kf->gain_scale = 1.0f;
    kf->measured_vector = allocate(z);
    kf->control_vector = u > 0 ? allocate(u) : nullptr;
    bind(kf->mat.xhat, kf->data.xhat, n, 1);
    bind(kf->mat.xhatminus, kf->data.xhatminus, n, 1);
    bind(kf->mat.z, kf->data.z, z, 1);
    bind(kf->mat.u, kf->data.u, u, 1);
    bind(kf->mat.b, kf->data.b, n, u);
    bind(kf->mat.a, kf->data.a, n, n);
    bind(kf->mat.at, kf->data.at, n, n);
    bind(kf->mat.h, kf->data.h, z, n);
    bind(kf->mat.ht, kf->data.ht, n, z);
    bind(kf->mat.p, kf->data.p, n, n);
    bind(kf->mat.pminus, kf->data.pminus, n, n);
    bind(kf->mat.q, kf->data.q, n, n);
    bind(kf->mat.r, kf->data.r, z, z);
    bind(kf->mat.k, kf->data.k, n, z);
    bind(kf->mat.k_denominator, kf->data.k_denominator, z, z);
    for (int index = 0; index < 2; ++index) {
        bind(kf->mat.cache_matrix[index], kf->data.cache_matrix[index], max_dim, max_dim);
        bind(kf->mat.cache_vector[index], kf->data.cache_vector[index], max_dim, 1);
    }
    kf->output = allocate(n);
    kf->chi_square_test = ChiSquareTest {};
    matrix::init(&kf->chi_square_test.chi_square_matrix, 1, 1, kf->chi_square_test.chi_square_data);
    kf->skip_step1 = kf->skip_step2 = kf->skip_step3 = false;
    kf->skip_step4 = kf->skip_step5 = false;
    kf->user_function0 = kf->user_function1 = kf->user_function2 = nullptr;
    kf->user_function3 = kf->user_function4 = kf->user_function5 = kf->user_function6 = nullptr;
    kf->mat_status = ARM_MATH_SUCCESS;
    kf->initialized = true;
    return true;
}

namespace {

void reshape(matrix::View& view, uint16_t rows, uint16_t cols) {
    view.numRows = rows;
    view.numCols = cols;
}

bool predictState(KalmanFilter* kf) {
    if (kf->skip_step1) {
        return true;
    }
    if (kf->u_size == 0) {
        kf->mat_status = matrix::multiply(&kf->mat.a, &kf->mat.xhat, &kf->mat.xhatminus);
        return kf->mat_status == ARM_MATH_SUCCESS;
    }
    reshape(kf->mat.cache_vector[0], kf->xhat_size, 1);
    reshape(kf->mat.cache_vector[1], kf->xhat_size, 1);
    kf->mat_status = matrix::multiply(&kf->mat.a, &kf->mat.xhat, &kf->mat.cache_vector[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::multiply(&kf->mat.b, &kf->mat.u, &kf->mat.cache_vector[1]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status =
        matrix::add(&kf->mat.cache_vector[0], &kf->mat.cache_vector[1], &kf->mat.xhatminus);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

bool predictCovariance(KalmanFilter* kf) {
    if (kf->skip_step2) {
        return true;
    }
    kf->mat_status = matrix::transpose(&kf->mat.a, &kf->mat.at);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::multiply(&kf->mat.a, &kf->mat.p, &kf->mat.pminus);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    reshape(kf->mat.cache_matrix[0], kf->xhat_size, kf->xhat_size);
    kf->mat_status = matrix::multiply(&kf->mat.pminus, &kf->mat.at, &kf->mat.cache_matrix[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::add(&kf->mat.cache_matrix[0], &kf->mat.q, &kf->mat.pminus);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

bool correctState(KalmanFilter* kf) {
    if (kf->skip_step4) {
        return true;
    }
    reshape(kf->mat.cache_vector[0], kf->z_size, 1);
    reshape(kf->mat.cache_vector[1], kf->z_size, 1);
    kf->mat_status = matrix::multiply(&kf->mat.h, &kf->mat.xhatminus, &kf->mat.cache_vector[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status =
        matrix::subtract(&kf->mat.z, &kf->mat.cache_vector[0], &kf->mat.cache_vector[1]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    reshape(kf->mat.cache_vector[0], kf->xhat_size, 1);
    kf->mat_status =
        matrix::multiply(&kf->mat.k, &kf->mat.cache_vector[1], &kf->mat.cache_vector[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::add(&kf->mat.xhatminus, &kf->mat.cache_vector[0], &kf->mat.xhat);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

bool correctCovariance(KalmanFilter* kf) {
    if (kf->skip_step5) {
        return true;
    }
    reshape(kf->mat.cache_matrix[0], kf->xhat_size, kf->xhat_size);
    reshape(kf->mat.cache_matrix[1], kf->xhat_size, kf->xhat_size);
    kf->mat_status = matrix::multiply(&kf->mat.k, &kf->mat.h, &kf->mat.cache_matrix[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status =
        matrix::multiply(&kf->mat.cache_matrix[0], &kf->mat.pminus, &kf->mat.cache_matrix[1]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::subtract(&kf->mat.pminus, &kf->mat.cache_matrix[1], &kf->mat.p);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

} // namespace

bool kalmanFilterInnovation(KalmanFilter* kf) {
    kf->mat_status = matrix::transpose(&kf->mat.h, &kf->mat.ht);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    reshape(kf->mat.cache_matrix[0], kf->z_size, kf->xhat_size);
    reshape(kf->mat.cache_matrix[1], kf->z_size, kf->z_size);
    kf->mat_status = matrix::multiply(&kf->mat.h, &kf->mat.pminus, &kf->mat.cache_matrix[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status =
        matrix::multiply(&kf->mat.cache_matrix[0], &kf->mat.ht, &kf->mat.cache_matrix[1]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::add(&kf->mat.cache_matrix[1], &kf->mat.r, &kf->mat.k_denominator);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status = matrix::inverse(&kf->mat.k_denominator, &kf->mat.cache_matrix[1]);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

bool kalmanFilterGain(KalmanFilter* kf) {
    reshape(kf->mat.cache_matrix[0], kf->xhat_size, kf->z_size);
    kf->mat_status = matrix::multiply(&kf->mat.pminus, &kf->mat.ht, &kf->mat.cache_matrix[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return false;
    }
    kf->mat_status =
        matrix::multiply(&kf->mat.cache_matrix[0], &kf->mat.cache_matrix[1], &kf->mat.k);
    return kf->mat_status == ARM_MATH_SUCCESS;
}

float* kalmanFilterUpdate(KalmanFilter* kf) {
#if LFCORE_PROFILE
    profiling::CycleScope cycle_scope { profiling::kalman_cycle_stats };
#endif
    if (kf == nullptr || !kf->initialized) {
        return nullptr;
    }
    kf->mat_status = ARM_MATH_SUCCESS;
    std::memcpy(kf->data.z, kf->measured_vector, sizeof(float) * kf->z_size);
    std::fill_n(kf->measured_vector, kf->z_size, 0.0f);
    if (kf->u_size > 0) {
        std::memcpy(kf->data.u, kf->control_vector, sizeof(float) * kf->u_size);
    }
    auto callback = [kf](KalmanFilter::Callback function) {
        if (function != nullptr) {
            function(kf);
        }
        return kf->mat_status == ARM_MATH_SUCCESS;
    };
    // Stop at the first failing operation; a later operation must not hide the error.
    if (!callback(kf->user_function0) || !predictState(kf) || !callback(kf->user_function1)
        || !predictCovariance(kf) || !callback(kf->user_function2)
        || (!kf->skip_step3 && (!kalmanFilterInnovation(kf) || !kalmanFilterGain(kf)))
        || !callback(kf->user_function3) || !correctState(kf) || !callback(kf->user_function4)
        || !correctCovariance(kf) || !callback(kf->user_function5) || !callback(kf->user_function6))
    {
        return kf->output;
    }
    std::memcpy(kf->output, kf->data.xhat, sizeof(float) * kf->xhat_size);
    return kf->output;
}
