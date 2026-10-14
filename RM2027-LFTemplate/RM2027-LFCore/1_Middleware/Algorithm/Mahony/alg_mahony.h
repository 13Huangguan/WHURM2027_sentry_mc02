/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : alg_mahony.h
  * @brief          : Mahony AHRS explicit complementary filter
  * @author         : Ported from IMU_based_on_stm32H7-main (MahonyAHRS.c/h)
  * @date           : 2026/10/13
  * @version        : v1.0
  ******************************************************************************
  * @attention      : BMI088 is a 6-axis sensor with no magnetometer, so only the
  *                    accelerometer is fused. The accelerometer measures the
  *                    gravity direction, which constrains the tilt (2 DOF) but
  *                    says nothing about rotation about gravity, so yaw is
  *                    unobservable and its gyro bias cannot be estimated here.
  *                    Keep the gyro Z offset accurate in Bmi088.c instead.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ALG_MAHONY_H
#define ALG_MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported macros -----------------------------------------------------------*/
/**
 * @brief proportional gain of the complementary filter.
 *        Larger pulls the attitude towards the accelerometer faster.
 */
#define Mahony_Kp  2.0f

/**
 * @brief integral gain, corrects the gyro bias online.
 *        Only the tilt axes are observable from the accelerometer, so this
 *        cannot fix the yaw bias. 0 disables the integral term entirely.
 */
#define Mahony_Ki  0.0f

/**
 * @brief the INS task loop runs at 1kHz, so one filter step is 1ms.
 *        EulerAngle[] is published in radians; convert with Config.h
 *        RadiansToDegrees where degrees are needed.
 */
#define Mahony_Sample_Period  0.001f

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the Mahony filter.
 */
typedef struct
{
  bool Init;   /*!< set once the filter has been seeded from a valid accel sample */

  float q0,q1,q2,q3;                     /*!< attitude quaternion */
  float integralErr_Bx, integralErr_By, integralErr_Bz;  /*!< integral feedback term */

  float invSampleFreq;  /*!< 1 / update rate, 0.001f at the 1kHz INS loop */

  /**
   * @brief Euler angles in radians, kept in the same order as the QuaternionEKF
   *        module used to produce them: [0] = yaw, [1] = pitch, [2] = roll.
   *        Read them through the IMU_ANGLE_INDEX_* macros in Config.h.
   */
  float EulerAngle[3];

} Mahony_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern Mahony_Info_Typedef Mahony_Info;

/* Exported function prototypes ---------------------------------------------*/
/**
 * @brief Seed the quaternion from a valid accelerometer sample.
 *        Only the tilt can be recovered without a magnetometer, so yaw starts
 *        at 0 by definition.
 */
extern void Mahony_Init(Mahony_Info_Typedef *Mahony, float ax, float ay, float az);

/**
 * @brief One complementary filter update. Gyro in rad/s, accel in any unit,
 *        it gets normalised internally.
 */
extern void Mahony_Update(Mahony_Info_Typedef *Mahony, float gx, float gy, float gz, float ax, float ay, float az);

/**
 * @brief Recompute EulerAngle[] from the current quaternion, in radians.
 */
extern void Mahony_ComputeAngles(Mahony_Info_Typedef *Mahony);

#ifdef __cplusplus
}
#endif

#endif //ALG_MAHONY_H
