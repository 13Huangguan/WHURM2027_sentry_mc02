/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_ins.h"

#include "alg_lpf.hpp"
#include "alg_quaternion.hpp"
#include "cmsis_os.h"
#include "dvc_bmi088.hpp"
#include "ins_config.hpp"
#include "lf_core.hpp"
#include "tim.h"

/**
  * @brief the structure that contains the information for the INS.
  */
InsInfo ins_info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float ins_lpf2p_alpha[3] = { 1.929454039488895f,
                                    -0.93178349823448126f,
                                    0.002329458745586203f };

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p ins_accel_pf2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  */
static float quaternion_ekf_a_data[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float quaternion_ekf_p_data[36] = { 100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   0.1f,   100000,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 0.1f,   100000, 0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 0.1f, 100000, 0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 0.1f, 100,  0.1f,   0.1f,   0.1f,
                                           0.1f,   0.1f, 0.1f, 100 };

TickType_t ins_task_sys_tick = 0;

/**
 * @brief Initializes the INS_Task.
 */
static void insTaskInit(void);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
extern "C" void INS_Task(void const* argument) {
    /* USER CODE BEGIN INS_Task */

    /* Initialize BMI088 IMU sensor - moved here from bspInit() to avoid HardFault
   * The sensor initialization requires Delay_ms() which uses HAL_GetTick().
   * This must be called after FreeRTOS scheduler has started to ensure proper timing. */
    static bool bmi088_initialized = false;
    if (!bmi088_initialized) {
        bmi088Init();
        bmi088_initialized = true;
    }

    /* Initializes the INS_Task. */
    insTaskInit();

    /* Infinite loop */
    for (;;) {
        ins_task_sys_tick = osKernelSysTick();

        /* Update the BMI088 measurement */
        bmi088InfoUpdate(&bmi088_info);

        /* Accel measurement LPF2p */
        ins_info.accel[0] = lowPassFilter2pUpdate(&ins_accel_pf2p[0], bmi088_info.accel[0]);
        ins_info.accel[1] = lowPassFilter2pUpdate(&ins_accel_pf2p[1], bmi088_info.accel[1]);
        ins_info.accel[2] = lowPassFilter2pUpdate(&ins_accel_pf2p[2], bmi088_info.accel[2]);

        /* Update the INS gyro in radians */
        ins_info.gyro[0] = bmi088_info.gyro[0];
        ins_info.gyro[1] = bmi088_info.gyro[1];
        ins_info.gyro[2] = bmi088_info.gyro[2];

        /* Update the QuaternionEKF */
        quaternionEkfUpdate(&quaternion_info, ins_info.gyro, ins_info.accel, 0.001f);

        memcpy(ins_info.angle, quaternion_info.euler_angle, sizeof(ins_info.angle));

        /* Update the Euler angle in degrees. */
        ins_info.pitch_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_PITCH] * BASIC_MATH_RAD_TO_DEG;
        ins_info.yaw_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_YAW] * BASIC_MATH_RAD_TO_DEG;
        ins_info.roll_angle =
            quaternion_info.euler_angle[IMU_ANGLE_INDEX_ROLL] * BASIC_MATH_RAD_TO_DEG;

        /* Update the yaw total angle */
        if (ins_info.yaw_angle - ins_info.last_yaw_angle < -180.f) {
            ins_info.yaw_round_count++;
        } else if (ins_info.yaw_angle - ins_info.last_yaw_angle > 180.f) {
            ins_info.yaw_round_count--;
        }
        ins_info.last_yaw_angle = ins_info.yaw_angle;

        ins_info.yaw_tol_angle = ins_info.yaw_angle + ins_info.yaw_round_count * 360.f;

        /* Update the INS gyro in degrees */
        ins_info.pitch_gyro = ins_info.gyro[IMU_GYRO_INDEX_PITCH] * RADIANS_TO_DEGREES;
        ins_info.yaw_gyro = ins_info.gyro[IMU_GYRO_INDEX_YAW] * RADIANS_TO_DEGREES;
        ins_info.roll_gyro = ins_info.gyro[IMU_GYRO_INDEX_ROLL] * RADIANS_TO_DEGREES;

        LuojiaFox::system_state.updateImu(
            { ins_info.yaw_angle,
              ins_info.pitch_angle,
              ins_info.roll_angle,
              ins_info.yaw_gyro,
              ins_info.pitch_gyro,
              ins_info.roll_gyro }
        );

        if (ins_task_sys_tick % 5 == 0) {
            // TODO: Add temperature control logic here
            // BMI088_Temp_Control(BMI088_Info.Temperature);
        }

        osDelayUntil(&ins_task_sys_tick, 1);
    }
    /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void insTaskInit(void) {
    ins_info.pitch_angle = 0.f;
    ins_info.yaw_angle = 0.f;
    ins_info.roll_angle = 0.f;
    ins_info.yaw_tol_angle = 0.f;
    ins_info.last_yaw_angle = 0.f;
    ins_info.yaw_round_count = 0;
    ins_info.pitch_gyro = 0.f;
    ins_info.yaw_gyro = 0.f;
    ins_info.roll_gyro = 0.f;
    ins_info.angle[0] = 0.f;
    ins_info.angle[1] = 0.f;
    ins_info.angle[2] = 0.f;
    ins_info.gyro[0] = 0.f;
    ins_info.gyro[1] = 0.f;
    ins_info.gyro[2] = 0.f;
    ins_info.accel[0] = 0.f;
    ins_info.accel[1] = 0.f;
    ins_info.accel[2] = 0.f;

    /* Initializes the Second order lowpass filter  */
    lowPassFilter2pInit(&ins_accel_pf2p[0], ins_lpf2p_alpha);
    lowPassFilter2pInit(&ins_accel_pf2p[1], ins_lpf2p_alpha);
    lowPassFilter2pInit(&ins_accel_pf2p[2], ins_lpf2p_alpha);

    /* Initializes the Quaternion EKF */
    quaternionEkfInit(
        &quaternion_info,
        10.f,
        0.001f,
        1000000.f,
        quaternion_ekf_a_data,
        quaternion_ekf_p_data
    );
}
//------------------------------------------------------------------------------
