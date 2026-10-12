/**
 * @file Motor.hpp
 * @author 
 * @brief 电机设备驱动（C++ 版本）
 * @version 1.0
 * @date 2026/10/04
 *
 * @attention 由 COD/Components/Device/Inc/Motor.h 转换而来，
 *            CAN 层沿用 bsp_can.h 的 FDCAN_TxFrame_TypeDef 接口。
 */

#ifndef DEVICE_MOTOR_HPP
#define DEVICE_MOTOR_HPP

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx.h"
#include "bsp_can.h"

/* Exported types ------------------------------------------------------------*/

/** 
* @brief 电机 CAN 收发标识符
*/
struct Motor_CANFrameInfo_typedef {
    uint32_t TxIdentifier;  /*!< 发送标识符 */
    uint32_t RxIdentifier;  /*!< 接收标识符 */
};


/** 
* @brief DJI 电机类型 
*/
enum DJI_Motor_Type_e {
    DJI_GM6020 = 0,  /*!< GM6020 */
    DJI_M3508,       /*!< M3508 */
    DJI_M2006,       /*!< M2006 */
    DJI_MOTOR_TYPE_NUM,     /*!< 类型总数 */
};


/** 
* @brief DJI 电机数据
*/
struct DJI_Motor_Data_Typedef{
    bool Initlized;     /*!< 初始化标志 */
    int16_t Current;      /*!< 电流 */
    int16_t Velocity;     /*!< 转速, rpm */
    int16_t Encoder;      /*!< 编码器原始值 */
    int16_t Last_Encoder;  /*!< 上次编码器值 */
    float Angle;          /*!< 角度, deg */
    uint8_t Temperature;  /*!< 温度 */
};


/**
* @brief DJI 电机（GM6020 / M3508 / M2006）
*/
class DJI_Motor_Info_Class {
public:
    DJI_Motor_Info_Class();

    /** @brief 配置电机类型与 CAN 收发标识符 */
    void Motor_Init(DJI_Motor_Type_e type, uint32_t txIdentifier, uint32_t rxIdentifier);

    /** @brief 用一帧 CAN 数据刷新电机状态 */
    void Motor_Update(uint32_t identifier, const uint8_t *rxBuffer);

    /**
     * @brief 把 4 台 DJI 电机的电流打包成一帧发送
     *
     * @param txFrame    已用 BSP_FDCAN_InitTxFrame 初始化过的发送帧
     * @param identifier 0x200（M3508/M2006 的 ID 1~4）、0x1FF（同型号 ID 5~8
     *                   或 GM6020 的 ID 1~4）、0x2FF（GM6020 的 ID 5~7）
     * @param current    4 台电机的电流，范围 -16384 ~ 16384
     */
    static void sendCurrent(FDCAN_TxFrame_TypeDef *txFrame, uint32_t identifier, const int16_t current[4]);

    bool isInitialized() const;
    DJI_Motor_Type_e getType() const;
    const  Motor_CANFrameInfo_typedef &getCanFrame() const;
    const  DJI_Motor_Data_Typedef &getData() const;

private:
    /** @brief 编码器值换算为累计角度, deg */
    float DJI_Motor_Encoder_To_Anglesum(float torqueRatio, uint16_t maxEncoder);

    /** @brief 编码器值换算为 -180~180 角度, deg */
    float DJI_Motor_Encoder_To_Angle(float torqueRatio, uint16_t maxEncoder);

    /** @brief 把角度循环限制到 [minValue, maxValue] */
    static float F_Loop_Constrain(float input, float minValue, float maxValue);

    DJI_Motor_Type_e type;
    Motor_CANFrameInfo_typedef canFrame;
    DJI_Motor_Data_Typedef data;
};







/** 
* @brief 达妙电机控制模式 
*/
enum DM_Motor_Control_Mode_Type_e {
    MIT = 0,           /*!< MIT 模式 */
    POSITION_VELOCITY,  /*!< 位置-速度模式 */
    VELOCITY,          /*!< 速度模式 */
};

/** @brief 达妙电机指令 */
enum DM_Motor_CMD_Type_e {
    Motor_Enable = 0,        /*!< 使能 */
    Motor_Disable,           /*!< 失能 */
    Motor_Save_Zero_Position,  /*!< 保存零点 */
    DM_Motor_CMD_Type_Num,               /*!< 指令总数 */
};

/** @brief 达妙电机参数范围 */
struct DM_Motor_Param_Range_Typedef{
    float P_MAX;  /*!< 位置范围, rad */
    float V_MAX;  /*!< 速度范围 */
    float T_MAX;  /*!< 扭矩范围 */
};

/** @brief 达妙电机数据 */
struct DM_Motor_Data_Typedef{
    bool Initlized;          /*!< 初始化标志 */
    uint8_t State;           /*!< 状态字 */
    uint16_t P_int;          /*!< 位置原始值 */
    uint16_t V_int;          /*!< 速度原始值 */
    uint16_t T_int;          /*!< 扭矩原始值 */
    float Position;          /*!< 位置, rad */
    float Velocity;          /*!< 速度 */
    float Torque;            /*!< 扭矩 */
    float Temperature_MOS;    /*!< MOS 温度 */
    float Temperature_Rotor;  /*!< 线圈温度 */
    float Angle;             /*!< 角度, deg */
};

/** @brief 达妙电机控制量 */
struct DM_Motor_Contorl_Info_Typedef {
    float Position;  /*!< 目标位置, rad */
    float Velocity;  /*!< 目标速度 */
    float KP;        /*!< 位置增益 */
    float KD;        /*!< 速度增益 */
    float Torque;    /*!< 前馈扭矩 */
    float Angle;     /*!< 目标角度, deg */
};

/**
 * @brief 达妙电机（DM 系列）
 */
class DM_Motor_Info_Class {
public:
     DM_Motor_Info_Class();

    /** @brief 配置控制模式、参数范围与 CAN 收发标识符 */
    void init(DM_Motor_Control_Mode_Type_e controlMode,
              const DM_Motor_Param_Range_Typedef &paramRange,
              uint32_t txIdentifier,
              uint32_t rxIdentifier);

    /** @brief 用一帧 CAN 数据刷新电机状态 */
    void DM_Motor_Info_Update(uint32_t identifier, const uint8_t *rxBuffer);

    /** @brief 发送使能/失能/保存零点指令 */
    void DM_Motor_Command(FDCAN_TxFrame_TypeDef *txFrame, DM_Motor_CMD_Type_e command);

    /** @brief MIT 模式发送控制量 */
    void DM_Motor_MIT_Send(FDCAN_TxFrame_TypeDef *txFrame, const DM_Motor_Contorl_Info_Typedef &control);

    /** @brief 位置-速度模式发送控制量 */
    void DM_Motor_POSITION_VELOCITY_Send(FDCAN_TxFrame_TypeDef *txFrame, float position, float velocity);

    /** @brief 速度模式发送控制量 */
    void DM_Motor_VELOCITY_Send(FDCAN_TxFrame_TypeDef *txFrame, float velocity);

    bool isInitialized() const;
    DM_Motor_Control_Mode_Type_e getControlMode() const;
    const Motor_CANFrameInfo_typedef &getCanFrame() const;
    const DM_Motor_Param_Range_Typedef &getParamRange() const;
    const DM_Motor_Data_Typedef &getData() const;

private:
    /** @brief 原始整数还原为浮点量 */
    static float uintToFloat(int xInt, float xMin, float xMax, int bits);

    /** @brief 浮点量映射为原始整数 */
    static int floatToUint(float x, float xMin, float xMax, int bits);

    DM_Motor_Control_Mode_Type_e controlMode;
    Motor_CANFrameInfo_typedef canFrame;
    DM_Motor_Param_Range_Typedef paramRange;
    DM_Motor_Data_Typedef data;
};

#endif  // DEVICE_MOTOR_HPP
