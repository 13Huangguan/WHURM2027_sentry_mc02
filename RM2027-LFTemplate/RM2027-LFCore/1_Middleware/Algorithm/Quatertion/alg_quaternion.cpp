#include "alg_quaternion.hpp"

#include "alg_basic.hpp"
#include "alg_kalman_filter.hpp"
#include "arm_math.h"
#include "ins_config.hpp"
#include "prf_cycle_counter.hpp"
#include <cmath>
#include <cstring>

static void quaternionEkfAUpdate(KalmanFilter* kf);

static void quaternionEkfHUpdate(KalmanFilter* kf);

static bool quaternionEkfChiSqrtTest(KalmanFilter* kf);

static void quaternionEkfXhatUpdate(KalmanFilter* kf);

QuaternionInfo quaternion_info;

void quaternionEkfInit(
    QuaternionInfo* quat,
    float process_noise1,
    float process_noise2,
    float measure_noise,
    const float* quaternion_ekf_a_data,
    const float* quaternion_ekf_p_data
) {
    if (quat == nullptr || quaternion_ekf_a_data == nullptr || quaternion_ekf_p_data == nullptr) {
        return;
    }
    quat->q1 = process_noise1;
    quat->q2 = process_noise2;
    quat->r = measure_noise;

    quat->quaternion_ekf_a_data = quaternion_ekf_a_data;
    quat->quaternion_ekf_p_data = quaternion_ekf_p_data;

    quat->initialized = false;
    if (!kalmanFilterInit(&quat->quaternion_ekf, 6, 0, 3)) {
        return;
    }

    quat->quaternion_ekf.chi_square_test.test_flag = false;
    quat->quaternion_ekf.chi_square_test.result = false;
    quat->quaternion_ekf.chi_square_test.chi_square_test_thresholds = 1e-8f;
    quat->quaternion_ekf.chi_square_test.chi_square_cnt = 0;

    quat->quaternion_ekf.data.xhat[0] = 1.f;
    quat->quaternion_ekf.data.xhat[1] = 0.f;
    quat->quaternion_ekf.data.xhat[2] = 0.f;
    quat->quaternion_ekf.data.xhat[3] = 0.f;

    quat->quaternion_ekf.user_function1 = quaternionEkfAUpdate;
    quat->quaternion_ekf.user_function2 = quaternionEkfHUpdate;
    quat->quaternion_ekf.user_function3 = quaternionEkfXhatUpdate;

    quat->quaternion_ekf.skip_step3 = true;
    quat->quaternion_ekf.skip_step4 = true;

    memcpy(
        quat->quaternion_ekf.data.a,
        quat->quaternion_ekf_a_data,
        quat->quaternion_ekf.sizeof_float * quat->quaternion_ekf.xhat_size
            * quat->quaternion_ekf.xhat_size
    );
    memcpy(
        quat->quaternion_ekf.data.p,
        quat->quaternion_ekf_p_data,
        quat->quaternion_ekf.sizeof_float * quat->quaternion_ekf.xhat_size
            * quat->quaternion_ekf.xhat_size
    );

    quat->euler_angle[0] = 0.f;
    quat->euler_angle[1] = 0.f;
    quat->euler_angle[2] = 0.f;

    quat->quat[0] = 1.f;
    quat->quat[1] = 0.f;
    quat->quat[2] = 0.f;
    quat->quat[3] = 0.f;

    quat->deviate[0] = 0.f;
    quat->deviate[1] = 0.f;
    quat->deviate[2] = 0.f;

    quat->accel[0] = 0.f;
    quat->accel[1] = 0.f;
    quat->accel[2] = 0.f;

    quat->gyro[0] = 0.f;
    quat->gyro[1] = 0.f;
    quat->gyro[2] = 0.f;

    quat->initialized = true;
}

void quaternionEkfUpdate(
    QuaternionInfo* quat,
    const float gyro[3],
    const float accel[3],
    float dt
) {
#if LFCORE_PROFILE
    profiling::CycleScope cycle_scope { profiling::quaternion_cycle_stats };
#endif
    if (quat == nullptr || !quat->initialized || gyro == nullptr || accel == nullptr
        || !std::isfinite(dt) || dt <= 0.0f)
    {
        return;
    }
    const float accel_squared_norm =
        accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
    if (!std::isfinite(accel_squared_norm) || accel_squared_norm <= 0.0f) {
        return;
    }
    for (int index = 0; index < 3; ++index) {
        if (!std::isfinite(gyro[index])) {
            return;
        }
    }
    /* update cycle */
    quat->quaternion_ekf.dt = dt;

    /* Apply accelerometer feedback to gyroscope */
    quat->gyro[0] = gyro[0] - quat->deviate[0];
    quat->gyro[1] = gyro[1] - quat->deviate[1];
    quat->gyro[2] = gyro[2] - quat->deviate[2];

    const float gyro_squared_norm = quat->gyro[0] * quat->gyro[0] + quat->gyro[1] * quat->gyro[1]
        + quat->gyro[2] * quat->gyro[2];

    /* Convert gyroscope to radians per second scaled by 0.5 */
    quat->half_gyro_dt[0] = 0.5f * quat->gyro[0] * quat->quaternion_ekf.dt;
    quat->half_gyro_dt[1] = 0.5f * quat->gyro[1] * quat->quaternion_ekf.dt;
    quat->half_gyro_dt[2] = 0.5f * quat->gyro[2] * quat->quaternion_ekf.dt;

    /**
   * @brief A = \frac{\partial f}{\partial x}
   *        (1,        -halfgxdt,  -halfgydt,  -halfgzdt,)   0.5f*q1*dt,  0.5f*q2*dt
   *        (halfgxdt,  1,          halfgzdt,  -halfgydt,)  -0.5f*q0*dt,  0.5f*q3*dt
   *        (halfgydt, -halfgzdt,   1,          halfgxdt,)  -0.5f*q3*dt, -0.5f*q0*dt
   *        (halfgzdt,  halfgydt,  -halfgxdt,   1,       )   0.5f*q2*dt, -0.5f*q1*dt
   *        (0,         0,          0,          0,         1,            0 )
   *        (0,         0,          0,          0,         0,            1 )
 */

    memcpy(
        quat->quaternion_ekf.data.a,
        quat->quaternion_ekf_a_data,
        quat->quaternion_ekf.sizeof_float * quat->quaternion_ekf.xhat_size
            * quat->quaternion_ekf.xhat_size
    );

    quat->quaternion_ekf.data.a[1] = -quat->half_gyro_dt[0];
    quat->quaternion_ekf.data.a[2] = -quat->half_gyro_dt[1];
    quat->quaternion_ekf.data.a[3] = -quat->half_gyro_dt[2];

    quat->quaternion_ekf.data.a[6] = quat->half_gyro_dt[0];
    quat->quaternion_ekf.data.a[8] = quat->half_gyro_dt[2];
    quat->quaternion_ekf.data.a[9] = -quat->half_gyro_dt[1];

    quat->quaternion_ekf.data.a[12] = quat->half_gyro_dt[1];
    quat->quaternion_ekf.data.a[13] = -quat->half_gyro_dt[2];
    quat->quaternion_ekf.data.a[15] = quat->half_gyro_dt[0];

    quat->quaternion_ekf.data.a[18] = quat->half_gyro_dt[2];
    quat->quaternion_ekf.data.a[19] = quat->half_gyro_dt[1];
    quat->quaternion_ekf.data.a[20] = -quat->half_gyro_dt[0];

    memcpy(quat->accel, accel, sizeof(quat->accel));

    /* Calculate direction of gravity indicated by measurement */
    quat->accel_inv_norm = 1.0f / std::sqrt(accel_squared_norm);

    quat->quaternion_ekf.measured_vector[0] = quat->accel[0] * quat->accel_inv_norm;
    quat->quaternion_ekf.measured_vector[1] = quat->accel[1] * quat->accel_inv_norm;
    quat->quaternion_ekf.measured_vector[2] = quat->accel[2] * quat->accel_inv_norm;

    /* chi square test */
    if (gyro_squared_norm < 0.3f * 0.3f
        && accel_squared_norm > (GRAVITY_ACCEL - 0.5f) * (GRAVITY_ACCEL - 0.5f)
        && accel_squared_norm < (GRAVITY_ACCEL + 0.5f) * (GRAVITY_ACCEL + 0.5f))
    {
        quat->quaternion_ekf.chi_square_test.test_flag = true;
    } else {
        quat->quaternion_ekf.chi_square_test.test_flag = false;
    }

    /* update the process/measurement noise covariance matrix */
    quat->quaternion_ekf.data.q[0] = quat->q1 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.q[7] = quat->q1 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.q[14] = quat->q1 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.q[21] = quat->q1 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.q[28] = quat->q2 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.q[35] = quat->q2 * quat->quaternion_ekf.dt;
    quat->quaternion_ekf.data.r[0] = quat->r;
    quat->quaternion_ekf.data.r[4] = quat->r;
    quat->quaternion_ekf.data.r[8] = quat->r;

    /* update the kalman filter */
    kalmanFilterUpdate(&quat->quaternion_ekf);
    if (quat->quaternion_ekf.mat_status != ARM_MATH_SUCCESS) {
        return;
    }

    /* Update the Euler angle in radians */
    quat->quat[0] = quat->quaternion_ekf.output[0];
    quat->quat[1] = quat->quaternion_ekf.output[1];
    quat->quat[2] = quat->quaternion_ekf.output[2];
    quat->quat[3] = quat->quaternion_ekf.output[3];
    quat->deviate[0] = quat->quaternion_ekf.output[4];
    quat->deviate[1] = quat->quaternion_ekf.output[5];
    quat->deviate[2] = 0.f;

    /* Update the Euler angle in radians */
    float yaw = atan2f(
        2.f * (quat->quat[0] * quat->quat[3] + quat->quat[1] * quat->quat[2]),
        1.f - 2.f * (quat->quat[2] * quat->quat[2] + quat->quat[3] * quat->quat[3])
    );
    float pitch = asinf(
        basic_math::constrain(
            2.f * (quat->quat[0] * quat->quat[2]) - 2.f * (quat->quat[1] * quat->quat[3]),
            -1.0f,
            1.0f
        )
    );
    float roll = atan2f(
        2.f * (quat->quat[0] * quat->quat[1] + quat->quat[2] * quat->quat[3]),
        1.f - 2.f * (quat->quat[1] * quat->quat[1] + quat->quat[2] * quat->quat[2])
    );

    quat->euler_angle[0] = (yaw != yaw) ? 0.f : yaw;
    quat->euler_angle[1] = (pitch != pitch) ? 0.f : pitch;
    quat->euler_angle[2] = (roll != roll) ? 0.f : roll;
}

static void quaternionEkfAUpdate(KalmanFilter* kf) {
    /* Normalise quaternion */
    memset(kf->data.cache_vector[0], 0, kf->sizeof_float * kf->xhat_size);
    kf->data.cache_vector[0][0] = basic_math::inverseSqrt(
        kf->data.xhatminus[0] * kf->data.xhatminus[0]
        + kf->data.xhatminus[1] * kf->data.xhatminus[1]
        + kf->data.xhatminus[2] * kf->data.xhatminus[2]
        + kf->data.xhatminus[3] * kf->data.xhatminus[3]
    );

    kf->data.xhatminus[0] *= kf->data.cache_vector[0][0];
    kf->data.xhatminus[1] *= kf->data.cache_vector[0][0];
    kf->data.xhatminus[2] *= kf->data.cache_vector[0][0];
    kf->data.xhatminus[3] *= kf->data.cache_vector[0][0];

    kf->data.a[4] = 0.5f * kf->data.xhatminus[1] * kf->dt;
    kf->data.a[5] = 0.5f * kf->data.xhatminus[2] * kf->dt;

    kf->data.a[10] = -0.5f * kf->data.xhatminus[0] * kf->dt;
    kf->data.a[11] = 0.5f * kf->data.xhatminus[3] * kf->dt;

    kf->data.a[16] = -0.5f * kf->data.xhatminus[3] * kf->dt;
    kf->data.a[17] = -0.5f * kf->data.xhatminus[0] * kf->dt;

    kf->data.a[22] = 0.5f * kf->data.xhatminus[2] * kf->dt;
    kf->data.a[23] = -0.5f * kf->data.xhatminus[1] * kf->dt;

    /* Limit the P data */
    kf->data.p[28] = basic_math::constrain<float>(kf->data.p[28], -10000, 10000);
    kf->data.p[35] = basic_math::constrain<float>(kf->data.p[35], -10000, 10000);
}

static void quaternionEkfHUpdate(KalmanFilter* kf) {
    memset(kf->data.h, 0, kf->sizeof_float * kf->z_size * kf->xhat_size);

    kf->data.h[0] = -2.f * kf->data.xhatminus[2];
    kf->data.h[1] = 2.f * kf->data.xhatminus[3];
    kf->data.h[2] = -2.f * kf->data.xhatminus[0];
    kf->data.h[3] = 2.f * kf->data.xhatminus[1];

    kf->data.h[6] = 2.f * kf->data.xhatminus[1];
    kf->data.h[7] = 2.f * kf->data.xhatminus[0];
    kf->data.h[8] = 2.f * kf->data.xhatminus[3];
    kf->data.h[9] = 2.f * kf->data.xhatminus[2];

    kf->data.h[12] = 2.f * kf->data.xhatminus[0];
    kf->data.h[13] = -2.f * kf->data.xhatminus[1];
    kf->data.h[14] = -2.f * kf->data.xhatminus[2];
    kf->data.h[15] = 2.f * kf->data.xhatminus[3];
}

static bool quaternionEkfChiSqrtTest(KalmanFilter* kf) {
    // Cache_Matrix[0] = inverse(H��Pminus��HT + R)��(z(k) - h(Xhatminus))
    kf->gain_scale = 1.0f;
    kf->mat.cache_matrix[0].numRows = kf->mat.cache_matrix[1].numRows;
    kf->mat.cache_matrix[0].numCols = 1;
    kf->mat_status = matrix::multiply(
        &kf->mat.cache_matrix[1],
        &kf->mat.cache_vector[1],
        &kf->mat.cache_matrix[0]
    );
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return true;
    }

    // Cache_Vector[0] = (z(k) - h(Xhatminus)'
    kf->mat.cache_vector[0].numRows = 1;
    kf->mat.cache_vector[0].numCols = kf->mat.cache_matrix[1].numRows;
    kf->mat_status = matrix::transpose(&kf->mat.cache_vector[1], &kf->mat.cache_vector[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return true;
    }

    // ChiSquare_Matrix = (z(k) - h(Xhatminus)'��inverse(H��Pminus��HT + R)��(z(k) - h(Xhatminus)) */
    kf->mat_status = matrix::multiply(
        &kf->mat.cache_vector[0],
        &kf->mat.cache_matrix[0],
        &kf->chi_square_test.chi_square_matrix
    );
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return true;
    }

    if (kf->chi_square_test.chi_square_data[0]
        < 0.5f * kf->chi_square_test.chi_square_test_thresholds)
    {
        kf->chi_square_test.result = true;
    }

    if (kf->chi_square_test.chi_square_data[0] > kf->chi_square_test.chi_square_test_thresholds
        && kf->chi_square_test.result)
    {
        if (kf->chi_square_test.test_flag) {
            kf->chi_square_test.chi_square_cnt++;
        } else {
            kf->chi_square_test.chi_square_cnt = 0;
        }

        if (kf->chi_square_test.chi_square_cnt > 50) {
            kf->chi_square_test.result = 0;
            kf->skip_step5 = false; // step-5 is cov mat P updating
        } else {
            /* xhat(k) = xhat'(k) */
            /* P(k) = P'(k) */
            memcpy(kf->data.xhat, kf->data.xhatminus, kf->sizeof_float * kf->xhat_size);
            memcpy(kf->data.p, kf->data.pminus, kf->sizeof_float * kf->xhat_size * kf->xhat_size);

            /* skip the P update */
            kf->skip_step5 = true;
            return true;
        }
    } else {
        /* The smaller the rk , the greater the gain */
        if (kf->chi_square_test.chi_square_data[0]
                > 0.1f * kf->chi_square_test.chi_square_test_thresholds
            && kf->chi_square_test.result)
        {
            kf->gain_scale = (kf->chi_square_test.chi_square_test_thresholds
                              - kf->chi_square_test.chi_square_data[0])
                / (0.9f * kf->chi_square_test.chi_square_test_thresholds);
        } else {
            kf->gain_scale = 1.f;
        }

        kf->chi_square_test.chi_square_cnt = 0;
        kf->skip_step5 = false;
    }
    return false;
}

static void quaternionEkfXhatUpdate(KalmanFilter* kf) {
    /* HT */
    if (!kalmanFilterInnovation(kf)) {
        return;
    }

    kf->mat.cache_vector[0].numRows = kf->mat.h.numRows;
    kf->mat.cache_vector[0].numCols = 1;
    kf->data.cache_vector[0][0] = 2.f
        * (kf->data.xhatminus[1] * kf->data.xhatminus[3]
           - kf->data.xhatminus[0] * kf->data.xhatminus[2]);
    kf->data.cache_vector[0][1] = 2.f
        * (kf->data.xhatminus[0] * kf->data.xhatminus[1]
           + kf->data.xhatminus[2] * kf->data.xhatminus[3]);
    kf->data.cache_vector[0][2] = kf->data.xhatminus[0] * kf->data.xhatminus[0]
        - kf->data.xhatminus[1] * kf->data.xhatminus[1]
        - kf->data.xhatminus[2] * kf->data.xhatminus[2]
        + kf->data.xhatminus[3] * kf->data.xhatminus[3];

    /* the cosine of three axis orientation */
    float orientation_cosine[2];

    /* calculate the cosine of three axis orientation */
    for (uint8_t i = 0; i < 2; i++) {
        orientation_cosine[i] =
            acosf(basic_math::constrain(fabsf(kf->data.cache_vector[0][i]), 0.0f, 1.0f));
    }

    /* Cache_Vector[1] = z(k) - h(xhat'(k)) */
    kf->mat.cache_vector[1].numRows = kf->mat.z.numRows;
    kf->mat.cache_vector[1].numCols = 1;
    kf->mat_status =
        matrix::subtract(&kf->mat.z, &kf->mat.cache_vector[0], &kf->mat.cache_vector[1]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return;
    }

    /* Chi Square root Test */
    if (quaternionEkfChiSqrtTest(kf) == true) {
        return;
    }

    if (!kalmanFilterGain(kf)) {
        return;
    }

    /* The smaller the rk , the greater the gain */
    for (uint8_t i = 0; i < kf->mat.k.numCols * kf->mat.k.numRows; i++) {
        kf->data.k[i] *= kf->gain_scale;
    }

    /**
   * @brief K = \frac {P��minus��HT}{H��Pminus��HT + V��R��VT}
   *          = [  0,  1,  2,
   *               3,  4,  5,
   *               6,  7,  8,
   *               9, 10, 11, 
   *             (12, 13, 14,)
   *             (15, 16, 17,)]
   * @note  K[12..17] *=  cos(axis)/(PI/2.f)
   */
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            kf->data.k[i * 3 + j] *= orientation_cosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    /* Cache_Vector[0] = K(k)��(z(k) - H��xhat'(k)) */
    kf->mat.cache_vector[0].numRows = kf->mat.k.numRows;
    kf->mat.cache_vector[0].numCols = 1;
    kf->mat_status =
        matrix::multiply(&kf->mat.k, &kf->mat.cache_vector[1], &kf->mat.cache_vector[0]);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return;
    }

    if (kf->chi_square_test.result) {
        kf->data.cache_vector[0][4] = basic_math::constrain<float>(
            kf->data.cache_vector[0][4],
            -1e-2f * kf->dt,
            1e-2f * kf->dt
        );
        kf->data.cache_vector[0][5] = basic_math::constrain<float>(
            kf->data.cache_vector[0][5],
            -1e-2f * kf->dt,
            1e-2f * kf->dt
        );
    }
    kf->data.cache_vector[0][3] = 0;

    kf->mat_status = matrix::add(&kf->mat.xhatminus, &kf->mat.cache_vector[0], &kf->mat.xhat);
    if (kf->mat_status != ARM_MATH_SUCCESS) {
        return;
    }
}
