#ifndef ALG_QUATERNION_HPP
#define ALG_QUATERNION_HPP

#include "alg_kalman_filter.hpp"

struct QuaternionInfo {
    bool initialized = false;

    float quat[4] {};
    float deviate[3] {};

    float q1 = 0.0f, q2 = 0.0f, r = 0.0f;

    const float* quaternion_ekf_a_data =
        nullptr; /*!< pointer to the data of state transition matrix */
    const float* quaternion_ekf_p_data =
        nullptr; /*!< pointer to the data of posteriori covariance matrix */
    KalmanFilter quaternion_ekf; /*!< Extended Kalman Filter */

    float accel[3] {}; /*!< the data of accel measure */
    float gyro[3] {}; /*!< the data of gyro measure */
    float accel_inv_norm = 0.0f; /*!< the inverse of accel norm */
    float half_gyro_dt[3] {}; /*!< half gyro dt */
    float euler_angle[3] {}; /*!< Euler angles in radians: */
};

extern QuaternionInfo quaternion_info;

extern void quaternionEkfInit(
    QuaternionInfo* quat,
    float process_noise1,
    float process_noise2,
    float measure_noise,
    const float* quaternion_ekf_a_data,
    const float* quaternion_ekf_p_data
);

extern void
quaternionEkfUpdate(QuaternionInfo* quat, const float gyro[3], const float accel[3], float dt);

#endif
