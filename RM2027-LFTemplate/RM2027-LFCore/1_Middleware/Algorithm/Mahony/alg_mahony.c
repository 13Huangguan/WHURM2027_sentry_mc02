/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : alg_mahony.c
  * @brief          : Mahony AHRS explicit complementary filter
  * @author         : Ported from IMU_based_on_stm32H7-main (MahonyAHRS.c/h)
  * @date           : 2026/10/13
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "alg_mahony.h"
#include "math.h"

/**
  * @brief the Mahony filter instance used by the INS task.
  */
Mahony_Info_Typedef Mahony_Info;

/* Private function prototypes -----------------------------------------------*/
static float Mahony_InvSqrt(float x);

//------------------------------------------------------------------------------
/**
  * @brief  reciprocal square root
  * @param  x  the value to invert, must be > 0
  * @retval 1 / sqrt(x)
  */
static float Mahony_InvSqrt(float x)
{
  return 1.0f / sqrtf(x);
}
//------------------------------------------------------------------------------
/**
  * @brief  Seed the quaternion from a valid accelerometer sample
  * @param  Mahony  pointer to the Mahony filter structure
  * @param  ax/ay/az  accelerometer sample, any unit
  * @retval none
  */
void Mahony_Init(Mahony_Info_Typedef *Mahony, float ax, float ay, float az)
{
  float recipNorm;
  float init_pitch, init_roll;
  float cr2, cp2, sr2, sp2;
  float q0, q1, q2, q3;

  recipNorm = Mahony_InvSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  /* Without a magnetometer only the tilt can be recovered, so yaw is taken as
   * zero by definition and the corresponding terms drop out of the product. */
  init_pitch = atan2f(-ax, az);
  init_roll  = atan2f(ay, az);

  cr2 = cosf(init_roll  * 0.5f);
  cp2 = cosf(init_pitch * 0.5f);
  sr2 = sinf(init_roll  * 0.5f);
  sp2 = sinf(init_pitch * 0.5f);

  q0 = cr2 * cp2;
  q1 = sr2 * cp2;
  q2 = cr2 * sp2;
  q3 = -sr2 * sp2;

  recipNorm = Mahony_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  Mahony->q0 = q0 * recipNorm;
  Mahony->q1 = q1 * recipNorm;
  Mahony->q2 = q2 * recipNorm;
  Mahony->q3 = q3 * recipNorm;

  Mahony->integralErr_Bx = 0.0f;
  Mahony->integralErr_By = 0.0f;
  Mahony->integralErr_Bz = 0.0f;

  Mahony->invSampleFreq = Mahony_Sample_Period;
  Mahony->Init = true;
}
//------------------------------------------------------------------------------
/**
  * @brief  One complementary filter update
  * @param  Mahony  pointer to the Mahony filter structure
  * @param  gx/gy/gz  gyroscope sample in rad/s
  * @param  ax/ay/az  accelerometer sample, any unit (normalised internally)
  * @retval none
  */
void Mahony_Update(Mahony_Info_Typedef *Mahony, float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  /* The accelerometer is the only reference there is. Skip the correction on a
   * dead sample instead of feeding a zero vector into the normalisation. */
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm = Mahony_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* gravity direction predicted by the current attitude, half scaled */
    halfvx = Mahony->q1 * Mahony->q3 - Mahony->q0 * Mahony->q2;
    halfvy = Mahony->q0 * Mahony->q1 + Mahony->q2 * Mahony->q3;
    halfvz = Mahony->q0 * Mahony->q0 - 0.5f + Mahony->q3 * Mahony->q3;

    /* Cross product of the measured and the predicted gravity: the tilt error.
     * It is always perpendicular to gravity, so it can only ever correct
     * roll/pitch - the yaw bias is invisible here. */
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (Mahony_Ki > 0.0f)
    {
      Mahony->integralErr_Bx += Mahony_Ki * halfex * Mahony->invSampleFreq;
      Mahony->integralErr_By += Mahony_Ki * halfey * Mahony->invSampleFreq;
      Mahony->integralErr_Bz += Mahony_Ki * halfez * Mahony->invSampleFreq;
      gx += Mahony->integralErr_Bx;
      gy += Mahony->integralErr_By;
      gz += Mahony->integralErr_Bz;
    }
    else
    {
      Mahony->integralErr_Bx = 0.0f;
      Mahony->integralErr_By = 0.0f;
      Mahony->integralErr_Bz = 0.0f;
    }

    gx += Mahony_Kp * halfex;
    gy += Mahony_Kp * halfey;
    gz += Mahony_Kp * halfez;
  }

  /* integrate the corrected rate into the quaternion */
  gx *= (0.5f * Mahony->invSampleFreq);
  gy *= (0.5f * Mahony->invSampleFreq);
  gz *= (0.5f * Mahony->invSampleFreq);

  qa = Mahony->q0;
  qb = Mahony->q1;
  qc = Mahony->q2;
  Mahony->q0 += (-qb * gx - qc * gy - Mahony->q3 * gz);
  Mahony->q1 += ( qa * gx + qc * gz - Mahony->q3 * gy);
  Mahony->q2 += ( qa * gy - qb * gz + Mahony->q3 * gx);
  Mahony->q3 += ( qa * gz + qb * gy - qc * gx);

  recipNorm = Mahony_InvSqrt(Mahony->q0 * Mahony->q0 + Mahony->q1 * Mahony->q1 +
                             Mahony->q2 * Mahony->q2 + Mahony->q3 * Mahony->q3);
  Mahony->q0 *= recipNorm;
  Mahony->q1 *= recipNorm;
  Mahony->q2 *= recipNorm;
  Mahony->q3 *= recipNorm;
}
//------------------------------------------------------------------------------
/**
  * @brief  Recompute the Euler angles from the current quaternion
  * @param  Mahony  pointer to the Mahony filter structure
  * @retval none
  * @note   Same formulas and same output order as the QuaternionEKF module it
  *         replaces, so the IMU_ANGLE_INDEX_* mapping in Config.h still holds.
  */
void Mahony_ComputeAngles(Mahony_Info_Typedef *Mahony)
{
  float yaw   = atan2f(Mahony->q1 * Mahony->q2 + Mahony->q0 * Mahony->q3,
                       0.5f - Mahony->q2 * Mahony->q2 - Mahony->q3 * Mahony->q3);
  float pitch = asinf(-2.0f * (Mahony->q1 * Mahony->q3 - Mahony->q0 * Mahony->q2));
  float roll  = atan2f(Mahony->q0 * Mahony->q1 + Mahony->q2 * Mahony->q3,
                       0.5f - Mahony->q1 * Mahony->q1 - Mahony->q2 * Mahony->q2);

  /* asinf leaves its domain at gimbal lock and returns NaN; keep it from
   * poisoning everything downstream */
  Mahony->EulerAngle[0] = (yaw   != yaw) ? 0.0f : yaw;
  Mahony->EulerAngle[1] = (pitch != pitch) ? 0.0f : pitch;
  Mahony->EulerAngle[2] = (roll  != roll) ? 0.0f : roll;
}
//------------------------------------------------------------------------------
