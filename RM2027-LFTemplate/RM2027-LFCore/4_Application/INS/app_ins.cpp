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
#include "Quaternion.h"
#include "Config.h"
#include "LF.hpp"

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
  * @note  Only used by the QuaternionEKF solver, which the Mahony filter
  *        replaced. Kept so that switching back is a one-line change.
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
 * @brief the target temperature of the BMI088 in celsius.
 */
static float BMI088_Target_Temperature = 40.0f;

#define BMI088_GYRO_Z_DEADBAND  0.012f

static const float BMI088_Heater_PWM_Max = 18000.0f;

static PID TempCtrl_PID;

#define BMI088_Warm_Calib_Enable  0

#if BMI088_Warm_Calib_Enable

float Warm_Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};
float Warm_Gyro_Offset_Average[3] = {0.0f, 0.0f, 0.0f};

uint8_t Warm_Count = 0;

volatile uint8_t Warm_Calib_State = 0;
#endif

/**
 * @brief Initializes the INS_Task.
 */
static void insTaskInit(void);

/**
 * @brief Control the BMI088 temperature.
 * @param temp  measure of the BMI088 temperature.
 */
static void BMI088_Temp_Control(float temp);

#if BMI088_Warm_Calib_Enable
/**
 * @brief Drive the one-off warm gyro-bias calibration.
 */
static void BMI088_Warm_Calib_Update(void);
#endif

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
    ins_info.gyro[1]   =   bmi088_info.gyro[1];
    ins_info.gyro[2]   =   bmi088_info.gyro[2];

    /* yaw轴死区 */
    if (fabsf(ins_info.gyro[2]) < BMI088_GYRO_Z_DEADBAND)
    {
        INS_Info.Gyro[2] = 0;
    }
		
#if INS_USE_MAHONY
		/* Seed the Mahony filter from the first accelerometer sample */
    if (!Mahony_Info.Init)
    {
      Mahony_Init(&Mahony_Info, INS_Info.Accel[0], INS_Info.Accel[1], INS_Info.Accel[2]);
    }

		/* Update the Mahony complementary filter */
    Mahony_Update(&Mahony_Info, INS_Info.Gyro[0], INS_Info.Gyro[1], INS_Info.Gyro[2],
                  INS_Info.Accel[0], INS_Info.Accel[1], INS_Info.Accel[2]);
    Mahony_ComputeAngles(&Mahony_Info);

    memcpy(INS_Info.Angle, Mahony_Info.EulerAngle, sizeof(INS_Info.Angle));

		/* Update the Euler angle in degrees. */
    INS_Info.Pitch_Angle = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
#else
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
            /* Update the BMI088 temperature control */
			      BMI088_Temp_Control(BMI088_Info.Temperature);
        }
        #if BMI088_Warm_Calib_Enable
            BMI088_Warm_Calib_Update();
        #endif
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
    TempCtrl_PID.init(2000.0f, 2000.0f, 3.6f, 0.0f, BMI088_Heater_PWM_Max, BMI088_Heater_PWM_Max, 0.005f,
                    0.0f, 0.0f, 0.0f, 0.0f, PidDFirstDisable);

#if INS_USE_MAHONY
  Mahony_Init(&Mahony_Info,INS_Info.Accel[0],INS_Info.Accel[1],INS_Info.Accel[2]);
#else

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
static void BMI088_Temp_Control(float temp)
{
  TempCtrl_PID.setTarget(BMI088_Target_Temperature);
  TempCtrl_PID.setNow(temp);
  TempCtrl_PID.timCalculatePeriodElapsedCallback();

  float Output = TempCtrl_PID.getOut();

  VAL_LIMIT(Output, 0.0f, BMI088_Heater_PWM_Max);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)Output);
}
//------------------------------------------------------------------------------
/**
  * @brief  Drive the one-off warm gyro-bias calibration
  * @retval none
  */
#if BMI088_Warm_Calib_Enable
static void BMI088_Warm_Calib_Update(void)
{
  static const float    Temp_Band      = 0.02f;
  static const uint16_t Window_Samples = 5000;
  static const uint8_t  Window_Num     = 500;

  static uint16_t SampleCnt = 0;
  static float    Gyro_Sum[3] = {0.0f, 0.0f, 0.0f};

  if (Warm_Calib_State == 2)
  {
    return;   /* already done */
  }

  if ((fabsf(BMI088_Info.Temperature - BMI088_Target_Temperature) >= Temp_Band))
  {
    return;
  }

  Warm_Calib_State = 1;   /* sampling has started, for the debugger */

  Gyro_Sum[0] += BMI088_Info.Gyro[0];
  Gyro_Sum[1] += BMI088_Info.Gyro[1];
  Gyro_Sum[2] += BMI088_Info.Gyro[2];

  if (++SampleCnt >= Window_Samples)
  {
    Warm_Gyro_Offset[0] = Gyro_Sum[0] / (float)Window_Samples + BMI088_Info.Offsets_Gyro_X;
    Warm_Gyro_Offset[1] = Gyro_Sum[1] / (float)Window_Samples + BMI088_Info.Offsets_Gyro_Y;
    Warm_Gyro_Offset[2] = Gyro_Sum[2] / (float)Window_Samples + BMI088_Info.Offsets_Gyro_Z;

    Warm_Gyro_Offset_Average[0] += Warm_Gyro_Offset[0];
    Warm_Gyro_Offset_Average[1] += Warm_Gyro_Offset[1];
    Warm_Gyro_Offset_Average[2] += Warm_Gyro_Offset[2];
    Warm_Count++;

    if (Warm_Count >= Window_Num)
    {
      Warm_Gyro_Offset_Average[0] /= (float)Window_Num;
      Warm_Gyro_Offset_Average[1] /= (float)Window_Num;
      Warm_Gyro_Offset_Average[2] /= (float)Window_Num;
      BMI088_Info.Offsets_Gyro_X = Warm_Gyro_Offset_Average[0];
      BMI088_Info.Offsets_Gyro_Y = Warm_Gyro_Offset_Average[1];
      BMI088_Info.Offsets_Gyro_Z = Warm_Gyro_Offset_Average[2];
      Warm_Calib_State = 2;
    }

    Gyro_Sum[0] = 0.0f;
    Gyro_Sum[1] = 0.0f;
    Gyro_Sum[2] = 0.0f;
    SampleCnt = 0;
  }
}
#endif
//------------------------------------------------------------------------------
