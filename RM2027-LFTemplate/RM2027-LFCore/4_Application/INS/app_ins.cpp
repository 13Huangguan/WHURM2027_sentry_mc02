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
#include "cmsis_os.h"
#include "app_ins.h"
#include "Bmi088.h"
#include "alg_lpf.h"
#include "alg_pid.h"
#include "tim.h"
#include "Config.h"
#include "LF.hpp"

/* Attitude solver selection ------------------------------------------------*/
/**
 * @brief 选择解算方式
 *          0 = QuaternionEKF   (Algorithm/Quatertion/Quaternion.c)
 *          1 = Mahony          (Algorithm/Mahony/alg_mahony.c)
 */
#ifndef INS_USE_MAHONY
#define INS_USE_MAHONY  0
#endif

#if INS_USE_MAHONY
#include "alg_mahony.h"
#else
#include "Quaternion.h"
#endif

/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info; 

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];  

/**
  * @brief the Initialize data of state transition matrix.
  * @note  Only used by the QuaternionEKF solver, which the Mahony filter
  *        replaced. Kept so that switching back is a one-line change.
  */
static float QuaternionEKF_A_Data[36] __attribute__((unused)) =
                                      {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  * @note  Only used by the QuaternionEKF solver, see above.
  */
static float QuaternionEKF_P_Data[36] __attribute__((unused)) =
                                       {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

TickType_t INS_Task_SysTick = 0;

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
static void INS_Task_Init(void);

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
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */

  /* Initialize BMI088 IMU sensor - moved here from bspInit() to avoid HardFault
   * The sensor initialization requires Delay_ms() which uses HAL_GetTick().
   * This must be called after FreeRTOS scheduler has started to ensure proper timing. */
  static bool bmi088_initialized = false;
  if (!bmi088_initialized) {
    BMI088_Init();
    bmi088_initialized = true;
  }

	/* Initializes the INS_Task. */
	INS_Task_Init();

  /* Infinite loop */
  for(;;)
  {
    INS_Task_SysTick = osKernelSysTick();
		
		/* Update the BMI088 measurement */
    BMI088_Info_Update(&BMI088_Info);

    /* Accel measurement LPF2p */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);
		
    /* Update the INS gyro in radians */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];

    /* yaw轴死区 */
    if (fabsf(INS_Info.Gyro[2]) < BMI088_GYRO_Z_DEADBAND)
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
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);

    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

		/* Update the Euler angle in degrees. */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
#endif
		
		/* Update the yaw total angle */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
		
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;
		
    /* Update the INS gyro in degrees */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;

    LuojiaFox::system_state.UpdateImu({
      INS_Info.Yaw_Angle,
      INS_Info.Pitch_Angle,
      INS_Info.Roll_Angle,
      INS_Info.Yaw_Gyro,
      INS_Info.Pitch_Gyro,
      INS_Info.Roll_Gyro
    });
		
		if(INS_Task_SysTick%5 == 0)
		{
			/* Update the BMI088 temperature control */
			BMI088_Temp_Control(BMI088_Info.Temperature);
		}

#if BMI088_Warm_Calib_Enable
		BMI088_Warm_Calib_Update();
#endif

    osDelayUntil(&INS_Task_SysTick,1);
		
  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void)
{
  INS_Info.Pitch_Angle = 0.f;
  INS_Info.Yaw_Angle = 0.f;
  INS_Info.Roll_Angle = 0.f;
  INS_Info.Yaw_TolAngle = 0.f;
  INS_Info.Last_Yaw_Angle = 0.f;
  INS_Info.YawRoundCount = 0;
  INS_Info.Pitch_Gyro = 0.f;
  INS_Info.Yaw_Gyro = 0.f;
  INS_Info.Roll_Gyro = 0.f;
  INS_Info.Angle[0] = 0.f;
  INS_Info.Angle[1] = 0.f;
  INS_Info.Angle[2] = 0.f;
  INS_Info.Gyro[0] = 0.f;
  INS_Info.Gyro[1] = 0.f;
  INS_Info.Gyro[2] = 0.f;
  INS_Info.Accel[0] = 0.f;
  INS_Info.Accel[1] = 0.f;
  INS_Info.Accel[2] = 0.f;

  /* Initializes the Second order lowpass filter  */
  LowPassFilter2p_Init(&INS_AccelPF2p[0], INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1], INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2], INS_LPF2p_Alpha);

  TempCtrl_PID.init(2000.0f, 2000.0f, 3.6f, 0.0f, BMI088_Heater_PWM_Max, BMI088_Heater_PWM_Max, 0.005f,
                    0.0f, 0.0f, 0.0f, 0.0f, PidDFirstDisable);

#if INS_USE_MAHONY
  Mahony_Init(&Mahony_Info,INS_Info.Accel[0],INS_Info.Accel[1],INS_Info.Accel[2]);
#else
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
#endif
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
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
