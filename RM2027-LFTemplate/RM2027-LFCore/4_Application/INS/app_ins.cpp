/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_ins.cpp
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

#include "detect.h"

#include "alg_basic.hpp"
#include "alg_lpf.hpp"
#include "alg_pid.h"
#include "alg_quaternion.hpp"
#include "cmsis_os.h"
#include "dvc_bmi088.hpp"
#include "ins_config.hpp"
#include "lf_core.hpp"
#include "tim.h"

/* Attitude solver selection ------------------------------------------------*/
/**
 * @brief 选择解算方式
 *          0 = QuaternionEKF   (Algorithm/Quatertion/alg_quaternion.cpp)
 *          1 = Mahony          (Algorithm/Mahony/alg_mahony.c)
 */
#ifndef INS_USE_MAHONY
#define INS_USE_MAHONY 0
#endif

#if INS_USE_MAHONY
#include "alg_mahony.h"
#endif

/* Heater ------------------------------------------------------------------- */
/**
 * @brief 加热目标温度（摄氏度）
 */
#ifndef BMI088_Target_Temperature
#define BMI088_Target_Temperature 40.0f
#endif

/**
 * @brief 加热 PWM 上限，对应 TIM3_CH4 的 ARR 值
 */
#ifndef BMI088_Heater_PWM_Max
#define BMI088_Heater_PWM_Max 18000.0f
#endif

/**
 * @brief yaw 轴陀螺死区（rad/s），低于此值视为静止直接清零
 */
#ifndef BMI088_GYRO_Z_DEADBAND
#define BMI088_GYRO_Z_DEADBAND 0.012f
#endif

/**
 * @brief 暖机零偏标定开关（当前关闭）
 */
#ifndef BMI088_Warm_Calib_Enable
#define BMI088_Warm_Calib_Enable 0
#endif

/* --- 加热保护阈值 --------------------------------------------------------- */
/** 温度读数超出这个范围就认为传感器异常，立即断加热
 *  BMI088 量程 -40 ~ +125 ℃，取略窄一点的值 */
#define HEATER_TEMP_VALID_MIN   (-40.0f)
#define HEATER_TEMP_VALID_MAX   (125.0f)

/** 过温保护：超过 OVER 立即断电，回落到 RESUME 以下才恢复（带回差防抖动） */
#define HEATER_OVER_TEMP_C      (50.0f)
#define HEATER_RESUME_TEMP_C    (43.0f)

/** 满功率持续超过这个时间就判定为卡住（传感器脱落 / PID 失控），锁死断加热 */
#define HEATER_FULL_POWER_RATIO (0.9f)
#define HEATER_FULL_POWER_MS    (10000u)

/** 满功率锁死后的解除条件：温度真的升到目标附近，说明传感器是活的 */
#define HEATER_STALL_CLEAR_C    (BMI088_Target_Temperature - 1.0f)

/**
 * @brief 加热器保护状态，便于在调试器里查看
 */
typedef struct {
    bool over_temp_latched;    /*!< 过温锁存 */
    bool stall_latched;        /*!< 满功率超时锁存 */
    bool sensor_invalid;       /*!< 上一拍温度读数非法 */
    uint32_t full_power_since; /*!< 本轮满功率起始时刻，0 表示当前不满功率 */
    uint32_t invalid_cnt;      /*!< 累计非法读数次数，便于排查 */
} Bmi088HeaterGuard;

Bmi088HeaterGuard bmi088_heater_guard;

/* Globals ------------------------------------------------------------------*/
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

static PID TempCtrl_PID;

#if BMI088_Warm_Calib_Enable

float Warm_Gyro_Offset[3] = { 0.0f, 0.0f, 0.0f };
float Warm_Gyro_Offset_Average[3] = { 0.0f, 0.0f, 0.0f };

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

/**
 * @brief 立即关闭加热，用于各种保护分支
 */
static inline void BMI088_Heater_Off(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

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

    /* 加热控制的分频计数器。用独立计数器而不是 sys_tick % 5，
       避免 osDelayUntil 抖动导致 PID 的等效 dt 不准。 */
    uint32_t heater_div = 0;

    /* Infinite loop */
    for (;;) {
        ins_task_sys_tick = osKernelSysTick();

        /* Update the BMI088 measurement */
        bmi088InfoUpdate(&bmi088_info);
        detect_hook(DETECT_BMI_GYRO);
        detect_hook(DETECT_BMI_ACCEL);

        /* Accel measurement LPF2p */
        ins_info.accel[0] = lowPassFilter2pUpdate(&ins_accel_pf2p[0], bmi088_info.accel[0]);
        ins_info.accel[1] = lowPassFilter2pUpdate(&ins_accel_pf2p[1], bmi088_info.accel[1]);
        ins_info.accel[2] = lowPassFilter2pUpdate(&ins_accel_pf2p[2], bmi088_info.accel[2]);

        /* Update the INS gyro in radians */
        ins_info.gyro[0] = bmi088_info.gyro[0];
        ins_info.gyro[1] = bmi088_info.gyro[1];
        ins_info.gyro[2] = bmi088_info.gyro[2];

        /* yaw 轴死区 */
        if (fabsf(ins_info.gyro[IMU_GYRO_INDEX_YAW]) < BMI088_GYRO_Z_DEADBAND) {
            ins_info.gyro[IMU_GYRO_INDEX_YAW] = 0.0f;
        }

#if INS_USE_MAHONY
        /* Seed the Mahony filter from the first accelerometer sample */
        if (!Mahony_Info.Init) {
            Mahony_Init(&Mahony_Info, ins_info.accel[0], ins_info.accel[1], ins_info.accel[2]);
        }

        /* Update the Mahony complementary filter */
        Mahony_Update(
            &Mahony_Info,
            ins_info.gyro[0],
            ins_info.gyro[1],
            ins_info.gyro[2],
            ins_info.accel[0],
            ins_info.accel[1],
            ins_info.accel[2]
        );
        Mahony_ComputeAngles(&Mahony_Info);

        memcpy(ins_info.angle, Mahony_Info.EulerAngle, sizeof(ins_info.angle));

        /* Update the Euler angle in degrees. */
        ins_info.pitch_angle =
            Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH] * BASIC_MATH_RAD_TO_DEG;
        ins_info.yaw_angle = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_YAW] * BASIC_MATH_RAD_TO_DEG;
        ins_info.roll_angle = Mahony_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL] * BASIC_MATH_RAD_TO_DEG;
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
#endif

        /* ---- 以下与解算方式无关，两套算法共用 ---- */

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

        /* 温控每 5 ms 跑一次，与 PID 初始化时的 dt = 0.005f 对应 */
        if (++heater_div >= 5) {
            heater_div = 0;
            BMI088_Temp_Control(bmi088_info.temperature);
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

    /* 温控 PID：dt 必须和实际调用周期一致，否则 D 项会算错 */
    TempCtrl_PID.init(
        2000.0f,
        2000.0f,
        3.6f,
        0.0f,
        BMI088_Heater_PWM_Max,
        BMI088_Heater_PWM_Max,
        0.005f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        PidDFirstDisable
    );

    /* 保护状态清零，并且上电先断加热 */
    memset(&bmi088_heater_guard, 0, sizeof(bmi088_heater_guard));
    BMI088_Heater_Off();

#if INS_USE_MAHONY
    Mahony_Init(&Mahony_Info, ins_info.accel[0], ins_info.accel[1], ins_info.accel[2]);
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
#endif
}

//------------------------------------------------------------------------------
/**
 * @brief Control the BMI088 temperature.
 * @param temp  measure of the BMI088 temperature.
 *
 * @note 除了正常 PID，还带三重保护，任何一条触发都会立刻把加热关掉：
 *       1. 读数非法（NaN / 超量程）—— 传感器掉了或是 I2C 读挂了
 *       2. 过温（>50℃）—— 锁存，直到回落到 43℃ 以下
 *       3. 满功率连续超过 10s —— 判定卡住（PID 失控或传感器一直读低温），
 *          锁存到温度真的升到目标附近才解除
 */
static void BMI088_Temp_Control(float temp) {
    const uint32_t now = HAL_GetTick();

    /* --- 保护 1：读数合法性 ---------------------------------------------
       NaN 的任何比较都是 false，所以 (> min && < max) 一并把 NaN 挡掉 */
    if (!(temp > HEATER_TEMP_VALID_MIN && temp < HEATER_TEMP_VALID_MAX)) {
        bmi088_heater_guard.sensor_invalid = true;
        bmi088_heater_guard.invalid_cnt++;
        bmi088_heater_guard.full_power_since = 0;
        BMI088_Heater_Off();
        return;
    }
    bmi088_heater_guard.sensor_invalid = false;

    /* --- 保护 2：过温，带回差 ------------------------------------------- */
    if (temp > HEATER_OVER_TEMP_C) {
        bmi088_heater_guard.over_temp_latched = true;
    }
    if (bmi088_heater_guard.over_temp_latched) {
        if (temp > HEATER_RESUME_TEMP_C) {
            bmi088_heater_guard.full_power_since = 0;
            BMI088_Heater_Off();
            return;
        }
        bmi088_heater_guard.over_temp_latched = false; /* 已经降下来了 */
    }

    /* --- 保护 3：满功率超时锁存 ----------------------------------------- */
    if (bmi088_heater_guard.stall_latched) {
        /* 只有温度真的上来了才认为不是卡住，允许重新加热 */
        if (temp < HEATER_STALL_CLEAR_C) {
            bmi088_heater_guard.full_power_since = 0;
            BMI088_Heater_Off();
            return;
        }
        bmi088_heater_guard.stall_latched = false;
    }

    /* --- 正常 PID -------------------------------------------------------- */
    TempCtrl_PID.setTarget(BMI088_Target_Temperature);
    TempCtrl_PID.setNow(temp);
    TempCtrl_PID.timCalculatePeriodElapsedCallback();

    float output = TempCtrl_PID.getOut();
    /* 用 constrain 而不是老的 VAL_LIMIT，那个宏在合并后的 alg_basic 里已经没了 */
    output = basic_math::constrain(output, 0.0f, BMI088_Heater_PWM_Max);

    /* 满功率计时：连续 10s 顶格就锁死 */
    if (output >= BMI088_Heater_PWM_Max * HEATER_FULL_POWER_RATIO) {
        if (bmi088_heater_guard.full_power_since == 0) {
            bmi088_heater_guard.full_power_since = now;
        } else if ((now - bmi088_heater_guard.full_power_since) > HEATER_FULL_POWER_MS) {
            bmi088_heater_guard.stall_latched = true;
            bmi088_heater_guard.full_power_since = 0;
            BMI088_Heater_Off();
            return;
        }
    } else {
        bmi088_heater_guard.full_power_since = 0;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)output);
}
//------------------------------------------------------------------------------
/**
  * @brief  Drive the one-off warm gyro-bias calibration
  * @retval none
  */
#if BMI088_Warm_Calib_Enable
static void BMI088_Warm_Calib_Update(void) {
    static const float Temp_Band = 0.02f;
    static const uint16_t Window_Samples = 5000;
    static const uint8_t Window_Num = 500;

    static uint16_t SampleCnt = 0;
    static float Gyro_Sum[3] = { 0.0f, 0.0f, 0.0f };

    if (Warm_Calib_State == 2) {
        return; /* already done */
    }

    if ((fabsf(bmi088_info.temperature - BMI088_Target_Temperature) >= Temp_Band)) {
        return;
    }

    Warm_Calib_State = 1; /* sampling has started, for the debugger */

    Gyro_Sum[0] += bmi088_info.gyro[0];
    Gyro_Sum[1] += bmi088_info.gyro[1];
    Gyro_Sum[2] += bmi088_info.gyro[2];

    if (++SampleCnt >= Window_Samples) {
        Warm_Gyro_Offset[0] = Gyro_Sum[0] / (float)Window_Samples + bmi088_info.offsets_gyro_x;
        Warm_Gyro_Offset[1] = Gyro_Sum[1] / (float)Window_Samples + bmi088_info.offsets_gyro_y;
        Warm_Gyro_Offset[2] = Gyro_Sum[2] / (float)Window_Samples + bmi088_info.offsets_gyro_z;

        Warm_Gyro_Offset_Average[0] += Warm_Gyro_Offset[0];
        Warm_Gyro_Offset_Average[1] += Warm_Gyro_Offset[1];
        Warm_Gyro_Offset_Average[2] += Warm_Gyro_Offset[2];
        Warm_Count++;

        if (Warm_Count >= Window_Num) {
            Warm_Gyro_Offset_Average[0] /= (float)Window_Num;
            Warm_Gyro_Offset_Average[1] /= (float)Window_Num;
            Warm_Gyro_Offset_Average[2] /= (float)Window_Num;
            bmi088_info.offsets_gyro_x = Warm_Gyro_Offset_Average[0];
            bmi088_info.offsets_gyro_y = Warm_Gyro_Offset_Average[1];
            bmi088_info.offsets_gyro_z = Warm_Gyro_Offset_Average[2];
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
