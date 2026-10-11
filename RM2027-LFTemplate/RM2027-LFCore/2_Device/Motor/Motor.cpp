/**
 * @file Motor.cpp
 * @author 
 * @brief 电机设备驱动实现（C++ 版本）
 * @version 1.0
 * @date 2026/10/04
 *
 * @attention 由 COD/Components/Device/Src/Motor.c 转换而来。
 */

/* Includes ------------------------------------------------------------------*/

#include "Motor.hpp"

#include <math.h>

/* Private constants ---------------------------------------------------------*/

namespace {

/* 弧度转角度，等价于原 COD Config.h 中的 RadiansToDegrees */
constexpr float RadiansToDegrees = 57.295779513f;

}  // namespace

/* DJI 电机 ------------------------------------------------------------------*/

/**
 * @brief 构造函数，所有成员清零
 */
DJI_Motor_Info_Class::DJI_Motor_Info_Class()
    : type(DJI_GM6020), canFrame{0U, 0U}, data{} {
}

/**
 * @brief 配置电机类型与 CAN 收发标识符
 */
void DJI_Motor_Info_Class::Motor_Init(DJI_Motor_Type_e motorType, uint32_t txIdentifier,
                                      uint32_t rxIdentifier) {
    this->type = motorType;
    canFrame.TxIdentifier = txIdentifier;
    canFrame.RxIdentifier = rxIdentifier;
    data = {};
}

/**
 * @brief 用一帧 CAN 数据刷新电机状态
 */
void DJI_Motor_Info_Class::Motor_Update(uint32_t identifier, const uint8_t *rxBuffer) {
    if (identifier != canFrame.RxIdentifier) {
        return;
    }

    data.Temperature = rxBuffer[6];
    data.Encoder = (int16_t)((int16_t)rxBuffer[0] << 8 | (int16_t)rxBuffer[1]);
    data.Velocity = (int16_t)((int16_t)rxBuffer[2] << 8 | (int16_t)rxBuffer[3]);
    data.Current = (int16_t)((int16_t)rxBuffer[4] << 8 | (int16_t)rxBuffer[5]);

    switch (type) {
        case DJI_GM6020:
            data.Angle = DJI_Motor_Encoder_To_Angle(1.f, 8192);
            break;

        case DJI_M3508:
            data.Angle = DJI_Motor_Encoder_To_Angle(3591.f / 187.f, 8192);
            break;

        case DJI_M2006:
            data.Angle = DJI_Motor_Encoder_To_Angle(36.f, 8192);
            break;

        default:
            break;
    }
}

/**
 * @brief 把 4 台 DJI 电机的电流打包成一帧发送
 */
void DJI_Motor_Info_Class::sendCurrent(FDCAN_TxFrame_TypeDef *txFrame, uint32_t identifier,
                                       const int16_t current[4]) {
    txFrame->Header.Identifier = identifier;

    for (uint8_t i = 0; i < 4; i++) {
        txFrame->Data[2 * i] = (uint8_t)(current[i] >> 8);
        txFrame->Data[2 * i + 1] = (uint8_t)(current[i]);
    }

    USER_FDCAN_AddMessageToTxFifoQ(txFrame);
}

bool DJI_Motor_Info_Class::isInitialized() const {
    return data.Initlized;
}

DJI_Motor_Type_e DJI_Motor_Info_Class::getType() const {
    return type;
}

const Motor_CANFrameInfo_typedef &DJI_Motor_Info_Class::getCanFrame() const {
    return canFrame;
}

const DJI_Motor_Data_Typedef &DJI_Motor_Info_Class::getData() const {
    return data;
}

/**
 * @brief 把编码器值换算为累计角度，超过 180 度继续累加，torqueRatio：减速比；maxEncoder：电机编码器一圈的计数
 */
float DJI_Motor_Info_Class::DJI_Motor_Encoder_To_Anglesum(float torqueRatio, uint16_t maxEncoder) {
    float res1 = 0, res2 = 0;

    if (data.Initlized != true) {
        data.Last_Encoder = data.Encoder;
        data.Angle = 0;
        data.Initlized = true;
    }

    if (data.Encoder < data.Last_Encoder) {
        res1 = data.Encoder - data.Last_Encoder + maxEncoder;
    } else if (data.Encoder > data.Last_Encoder) {
        res1 = data.Encoder - data.Last_Encoder - maxEncoder;
    }
    res2 = data.Encoder - data.Last_Encoder;

    data.Last_Encoder = data.Encoder;

    if (fabsf(res1) > fabsf(res2)) {
        data.Angle += (float)res2 / (maxEncoder * torqueRatio) * 360.f;
    } else {
        data.Angle += (float)res1 / (maxEncoder * torqueRatio) * 360.f;
    }

    return data.Angle;
}

/**
 * @brief 把编码器值换算为 -180~180 角度
 */
float DJI_Motor_Info_Class::DJI_Motor_Encoder_To_Angle(float torqueRatio, uint16_t maxEncoder) {
    float encoderError = 0.f;

    if (data.Initlized != true) {
        data.Last_Encoder = data.Encoder;
        data.Angle = data.Encoder / (maxEncoder * torqueRatio) * 360.f;
        data.Initlized = true;
    }

    encoderError = data.Encoder - data.Last_Encoder;

    if (encoderError > maxEncoder * 0.5f) {
        data.Angle += (float)(encoderError - maxEncoder) / (maxEncoder * torqueRatio) * 360.f;
    } else if (encoderError < -maxEncoder * 0.5f) {
        data.Angle += (float)(encoderError + maxEncoder) / (maxEncoder * torqueRatio) * 360.f;
    } else {
        data.Angle += (float)(encoderError) / (maxEncoder * torqueRatio) * 360.f;
    }

    data.Last_Encoder = data.Encoder;

    data.Angle = F_Loop_Constrain(data.Angle, -180.f, 180.f);

    return data.Angle;
}

/**
 * @brief 把角度循环限制到 [minValue, maxValue]
 */
float DJI_Motor_Info_Class::F_Loop_Constrain(float input, float minValue, float maxValue) {
    if (maxValue < minValue) {
        return input;
    }

    float len = maxValue - minValue;

    if (input > maxValue) {
        do {
            input -= len;
        } while (input > maxValue);
    } else if (input < minValue) {
        do {
            input += len;
        } while (input < minValue);
    }

    return input;
}

/* 达妙电机 ------------------------------------------------------------------*/

/**
 * @brief 构造函数，所有成员清零
 */
DM_Motor_Info_Class::DM_Motor_Info_Class()
    : controlMode(MIT), canFrame{0U, 0U}, paramRange{}, data{} {
}

/**
 * @brief 配置控制模式、参数范围与 CAN 收发标识符
 */
void DM_Motor_Info_Class::init(DM_Motor_Control_Mode_Type_e mode,
                               const DM_Motor_Param_Range_Typedef &range,
                               uint32_t txIdentifier,
                               uint32_t rxIdentifier) {
    this->controlMode = mode;
    this->paramRange = range;
    canFrame.TxIdentifier = txIdentifier;
    canFrame.RxIdentifier = rxIdentifier;
    data = {};
}

/**
 * @brief 用一帧 CAN 数据刷新电机状态
 */
void DM_Motor_Info_Class::DM_Motor_Info_Update(uint32_t identifier, const uint8_t *rxBuffer) {
    if (identifier != canFrame.RxIdentifier) {
        return;
    }

    data.State = rxBuffer[0] >> 4;
    data.P_int = ((uint16_t)(rxBuffer[1]) << 8) | ((uint16_t)(rxBuffer[2]));
    data.V_int = ((uint16_t)(rxBuffer[3]) << 4) | ((uint16_t)(rxBuffer[4]) >> 4);
    data.T_int = ((uint16_t)(rxBuffer[4] & 0xF) << 8) | ((uint16_t)(rxBuffer[5]));

    data.Torque = uintToFloat(data.T_int, -paramRange.T_MAX, paramRange.T_MAX, 12);
    data.Position = uintToFloat(data.P_int, -paramRange.P_MAX, paramRange.P_MAX, 16);
    data.Velocity = uintToFloat(data.V_int, -paramRange.V_MAX, paramRange.V_MAX, 12);
    data.Angle = data.Position * RadiansToDegrees;

    data.Temperature_MOS = (float)(rxBuffer[6]);
    data.Temperature_Rotor = (float)(rxBuffer[7]);
}

bool DM_Motor_Info_Class::isInitialized() const {
    return data.Initlized;
}

DM_Motor_Control_Mode_Type_e DM_Motor_Info_Class::getControlMode() const {
    return controlMode;
}

const Motor_CANFrameInfo_typedef &DM_Motor_Info_Class::getCanFrame() const {
    return canFrame;
}

const DM_Motor_Param_Range_Typedef &DM_Motor_Info_Class::getParamRange() const {
    return paramRange;
}

const DM_Motor_Data_Typedef &DM_Motor_Info_Class::getData() const {
    return data;
}

/**
 * @brief 发送使能/失能/保存零点指令
 */
void DM_Motor_Info_Class::DM_Motor_Command(FDCAN_TxFrame_TypeDef *txFrame,
                                           DM_Motor_CMD_Type_e command) {
    txFrame->Header.Identifier = canFrame.RxIdentifier;

    txFrame->Data[0] = 0xFF;
    txFrame->Data[1] = 0xFF;
    txFrame->Data[2] = 0xFF;
    txFrame->Data[3] = 0xFF;
    txFrame->Data[4] = 0xFF;
    txFrame->Data[5] = 0xFF;
    txFrame->Data[6] = 0xFF;

    switch (command) {
        case Motor_Enable:
            txFrame->Data[7] = 0xFC;
            break;

        case Motor_Disable:
            txFrame->Data[7] = 0xFD;
            break;

        case Motor_Save_Zero_Position:
            txFrame->Data[7] = 0xFE;
            break;

        default:
            break;
    }

    USER_FDCAN_AddMessageToTxFifoQ(txFrame);
}

/**
 * @brief MIT 模式发送控制量
 */
void DM_Motor_Info_Class::DM_Motor_MIT_Send(FDCAN_TxFrame_TypeDef *txFrame,
                                            const DM_Motor_Contorl_Info_Typedef &control) {
    uint16_t positionTmp = (uint16_t)floatToUint(control.Position, -paramRange.P_MAX, paramRange.P_MAX, 16);
    uint16_t velocityTmp = (uint16_t)floatToUint(control.Velocity, -paramRange.V_MAX, paramRange.V_MAX, 12);
    uint16_t torqueTmp = (uint16_t)floatToUint(control.Torque, -paramRange.T_MAX, paramRange.T_MAX, 12);
    uint16_t kpTmp = (uint16_t)floatToUint(control.KP, 0, 500, 12);
    uint16_t kdTmp = (uint16_t)floatToUint(control.KD, 0, 5, 12);

    txFrame->Header.Identifier = canFrame.TxIdentifier;

    txFrame->Data[0] = (uint8_t)(positionTmp >> 8);
    txFrame->Data[1] = (uint8_t)(positionTmp);
    txFrame->Data[2] = (uint8_t)(velocityTmp >> 4);
    txFrame->Data[3] = (uint8_t)((velocityTmp & 0x0F) << 4) | (uint8_t)(kpTmp >> 8);
    txFrame->Data[4] = (uint8_t)(kpTmp);
    txFrame->Data[5] = (uint8_t)(kdTmp >> 4);
    txFrame->Data[6] = (uint8_t)((kdTmp & 0x0F) << 4) | (uint8_t)(torqueTmp >> 8);
    txFrame->Data[7] = (uint8_t)(torqueTmp);

    USER_FDCAN_AddMessageToTxFifoQ(txFrame);
}

/**
 * @brief 位置-速度模式发送控制量
 */
void DM_Motor_Info_Class::DM_Motor_POSITION_VELOCITY_Send(FDCAN_TxFrame_TypeDef *txFrame,
                                                          float position,
                                                          float velocity) {
    const uint8_t *positionTmp = reinterpret_cast<const uint8_t *>(&position);
    const uint8_t *velocityTmp = reinterpret_cast<const uint8_t *>(&velocity);

    txFrame->Header.Identifier = canFrame.TxIdentifier + 0x100;

    txFrame->Data[0] = positionTmp[0];
    txFrame->Data[1] = positionTmp[1];
    txFrame->Data[2] = positionTmp[2];
    txFrame->Data[3] = positionTmp[3];
    txFrame->Data[4] = velocityTmp[0];
    txFrame->Data[5] = velocityTmp[1];
    txFrame->Data[6] = velocityTmp[2];
    txFrame->Data[7] = velocityTmp[3];

    USER_FDCAN_AddMessageToTxFifoQ(txFrame);
}

/**
 * @brief 速度模式发送控制量
 */
void DM_Motor_Info_Class::DM_Motor_VELOCITY_Send(FDCAN_TxFrame_TypeDef *txFrame, float velocity) {
    const uint8_t *velocityTmp = reinterpret_cast<const uint8_t *>(&velocity);

    txFrame->Header.Identifier = canFrame.TxIdentifier + 0x200;

    txFrame->Data[0] = velocityTmp[0];
    txFrame->Data[1] = velocityTmp[1];
    txFrame->Data[2] = velocityTmp[2];
    txFrame->Data[3] = velocityTmp[3];
    txFrame->Data[4] = 0;
    txFrame->Data[5] = 0;
    txFrame->Data[6] = 0;
    txFrame->Data[7] = 0;

    USER_FDCAN_AddMessageToTxFifoQ(txFrame);
}

/**
 * @brief 把原始整数还原为浮点量
 */
float DM_Motor_Info_Class::uintToFloat(int xInt, float xMin, float xMax, int bits) {
    float span = xMax - xMin;
    float offset = xMin;

    return ((float)xInt) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 把浮点量映射为原始整数
 */
int DM_Motor_Info_Class::floatToUint(float x, float xMin, float xMax, int bits) {
    float span = xMax - xMin;
    float offset = xMin;

    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
