/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_motor.cpp
  * @brief          : 电机应用层
  * @version        : v1.0
  * @date           : 2026/10/04
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "app_motor.h"

#include "bsp_can.h"
#include "cmsis_os.h"
#include "fdcan.h"

#include "alg_pid.h"
#include "remote_control.h"

#include <math.h>

// #include "smc.h"
/* Private macros ------------------------------------------------------------*/

/* GM6020 收发 ID，按实际接线修改 */
#define GM6020_TX_ID   0x1FF
#define GM6020_RX_ID   0x205

/* 1=遥控器速度控制，0=原来的角度控制 */
#define GM6020_USE_REMOTE_SPEED 1

/* 遥控器速度控制参数 */
#define GM6020_RC_SPEED_CHANNEL RC_CH_RIGHT_VERTICAL
#define GM6020_RC_SPEED_MAX_RPM 300.0f
#define GM6020_RC_SPEED_DEADZONE 0.05f

/* 收到第一帧 GM6020 反馈后，相对当前位置转动 30 度；设为 0 表示只保持当前位置 */
#define PITCH_START_OFFSET_DEG 30.0f

/* Private variables ---------------------------------------------------------*/
//smc
// static Sliding speedSmc;

// static const fp32 speedSmcParam[9] = {
//     1.0f,    // J
//     5.0f,    // K
//     0.05f,   // c
//     0.0f,    // c1，VELSMC 不用
//     0.0f,    // c2，VELSMC 不用
//     0.0f,    // p，TFSMC 才用
//     0.0f,    // q，TFSMC 才用
//     0.0f,    // beta，TFSMC 才用
//     0.1f     // epsilon
// };





DJI_Motor_Info_Class g_pitchMotor;

/* GM6020 发送帧 */
static FDCAN_TxFrame_TypeDef txFrame1;

static PID   pitchAnglePid;  /* 外环：角度 -> 目标转速(rpm) */
static PID   pitchSpeedPid;  /* 内环：转速 -> 电压 */
static float targetAngle = 0.0f;   /* 想控制的云台角度 */
#if !GM6020_USE_REMOTE_SPEED
static bool  pitchHoldReady = false;  /* 软启动标志：收到第一帧反馈后把目标设为当前角度 */

/**
 * @brief 把角度折算到 -180~180 度，用于计算最短转向误差
 */
static float WrapDegrees180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}
#endif

/* Exported functions --------------------------------------------------------*/

void Motor_Init(void)
{
    /* 配置滤波器、启动三路 FDCAN、打开接收中断 */
    BSP_FDCAN_Init();

    /* 初始化 GM6020 发送帧，绑定 CAN1 */
    BSP_FDCAN_InitTxFrame(&txFrame1, &hfdcan1);

    /* GM6020 */
    g_pitchMotor.Motor_Init(DJI_GM6020, GM6020_TX_ID, GM6020_RX_ID);

    // SMC_init(
    // &speedSmc,
    // speedSmcParam,
    // VELSMC,     // 当前 GM6020 使用的是速度环
    // 1.0f,       // 饱和函数 limit
    // 6000.0f,    // 输出限幅 u_max，第一次测试先不要用 8000
    // 0.0f        // VELSMC 不使用位置误差阈值
    // );
    /* 外环：角度误差 -> 目标转速，输出限幅 50rpm（约 300 度/秒） */
    pitchAnglePid.init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 25.0f, 0.001f,
                       0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable);

    /* 内环：转速误差 -> 电压，输出限幅 8000（约 6.4V），积分限幅 4000 */
    pitchSpeedPid.init(50.0f, 5.0f, 0.0f, 0.0f, 4000.0f, 8000.0f, 0.001f,
                       0.0f, 0.0f, 0.0f, 0.0f, PidDFirstEnable);
    Motor_SetPitchAngle(targetAngle);
}

void Motor_Control(void)
{
#if GM6020_USE_REMOTE_SPEED
    float rcValue = 0.0f;
    float speedTarget = 0.0f;

    /* 只有遥控器在线且电机反馈正常时才允许输出速度指令 */
    if (!GetRcOffline() && g_pitchMotor.isInitialized()) {
        rcValue = GetDt7RcCh(GM6020_RC_SPEED_CHANNEL);
        if (fabsf(rcValue) < GM6020_RC_SPEED_DEADZONE) {
            rcValue = 0.0f;
        }
        speedTarget = rcValue * GM6020_RC_SPEED_MAX_RPM;
    } else {
        pitchSpeedPid.setIntegralError(0.0f);
    }

    pitchSpeedPid.setTarget(speedTarget);
    pitchSpeedPid.setNow(g_pitchMotor.getData().Velocity);
    pitchSpeedPid.timCalculatePeriodElapsedCallback();
    int16_t pitchOut = (int16_t)pitchSpeedPid.getOut();
    // int16_t pitchOut = 0;

    // if (!GetRcOffline() && g_pitchMotor.isInitialized()) {
    // SMC_volErrorUpdate(
    //     &speedSmc,
    //     speedTarget,                              // 目标转速，单位 rpm
    //     g_pitchMotor.getData().Velocity,          // 当前转速，单位 rpm
    //     0.001f                                    // 1 ms 控制周期
    // );

    // pitchOut = (int16_t)SMC_calc(&speedSmc);
    // } else {
    // SMC_intgValClear(&speedSmc);
    // speedSmc.u = 0.0f;
    // pitchOut = 0;
// }
#else
    /* 收到第一帧反馈后，把目标设成当前角度，避免上电猛冲 */
    if (!pitchHoldReady && g_pitchMotor.isInitialized()) {
        targetAngle = g_pitchMotor.getData().Angle + PITCH_START_OFFSET_DEG;
        pitchAnglePid.setTarget(targetAngle);
        pitchHoldReady = true;
    }
    /* 串级 PID：外环角度 -> 目标转速，内环转速 -> 电压 */
    const float currentAngle = g_pitchMotor.getData().Angle;
    const float angleError = WrapDegrees180(targetAngle - currentAngle);
    pitchAnglePid.setTarget(currentAngle + angleError);
    pitchAnglePid.setNow(currentAngle);
    pitchAnglePid.timCalculatePeriodElapsedCallback();

    pitchSpeedPid.setTarget(pitchAnglePid.getOut());
    pitchSpeedPid.setNow(g_pitchMotor.getData().Velocity);
    pitchSpeedPid.timCalculatePeriodElapsedCallback();

    int16_t pitchOut = (int16_t)pitchSpeedPid.getOut();
#endif

        /* ---- 云台 GM6020：单独一帧，电压控制 ---- */
    int16_t pitchVoltage[4] = {pitchOut, 0, 0, 0};   /* 一帧 4 台，只用第一台 */
    DJI_Motor_Info_Class::sendCurrent(&txFrame1, GM6020_TX_ID, pitchVoltage);
}

/**
  * @brief CAN 接收分发，由 bsp_can.c 的接收中断调用
  */
/**
  * @brief 设置云台 pitch 的目标角度
  * @param angle 目标角度，单位度
  */
void Motor_SetPitchAngle(float angle)
{
    targetAngle = angle;
    pitchAnglePid.setTarget(angle);
}

extern "C" void BSP_FDCAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t identifier, uint8_t *data)
{
    if (hcan == &hfdcan1)
    {
        g_pitchMotor.Motor_Update(identifier, data);
    }
}

/**
  * @brief CAN 任务：1kHz 周期发送电机控制量
  */
extern "C" void CAN_Task(void const *argument)
{
    (void)argument;

    for (;;)
    {
        Motor_Control();
        osDelay(1);
    }
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
