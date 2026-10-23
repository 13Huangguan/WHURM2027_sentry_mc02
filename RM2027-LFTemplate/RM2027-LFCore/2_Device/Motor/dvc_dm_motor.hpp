/**
 * @file dvc_dm_motor.hpp
 * @author LuojiaFox
 * @brief 达妙电机驱动
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.hpp"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机控制模式
 * @note 模式值同时是命令标识符的偏移量。
 */
enum class DmControlMode : uint16_t {
    Mit = 0x000,              /*!< MIT 模式 */
    PositionVelocity = 0x100, /*!< 位置-速度模式 */
    Velocity = 0x200,         /*!< 速度模式 */
};

/** @brief 达妙电机指令 */
enum class DmCommand : uint8_t {
    Enable,           /*!< 使能 */
    Disable,          /*!< 失能 */
    SaveZeroPosition, /*!< 保存当前位置为零点 */
};

/**
 * @brief 达妙电机参数范围
 * @note 需与上位机中的电机参数保持一致。
 */
struct DmParamRange {
    float position_max = 12.5f; /*!< 位置范围 ±rad */
    float velocity_max = 30.0f; /*!< 速度范围 ±rad/s */
    float torque_max = 10.0f;   /*!< 力矩范围 ±N*m */
};

/** @brief 达妙 MIT 控制量 */
struct DmControl {
    float position_rad = 0.0f;   /*!< 目标位置 */
    float velocity_rad_s = 0.0f; /*!< 目标速度 */
    float kp = 0.0f;             /*!< 位置增益，0~500 */
    float kd = 0.0f;             /*!< 速度增益，0~5 */
    float torque_nm = 0.0f;      /*!< 前馈力矩 */
};

/** @brief 达妙电机反馈数据 */
struct DmMotorData {
    uint8_t state = 0;              /*!< 状态字 */
    float position_rad = 0.0f;      /*!< 位置，rad */
    float velocity_rad_s = 0.0f;    /*!< 速度，rad/s */
    float torque_nm = 0.0f;         /*!< 力矩，N*m */
    float temperature_mos = 0.0f;   /*!< MOS 温度，摄氏度 */
    float temperature_rotor = 0.0f; /*!< 线圈温度，摄氏度 */
    bool initialized = false;       /*!< 是否收到过反馈 */
};

/**
 * @brief 达妙电机
 *
 * @note 命令标识符 = tx_identifier + 模式偏移（MIT 0x000 / 位置速度 0x100 / 速度 0x200）。
 *       反馈标识符由上位机中的 master ID 决定，按参考工程约定为 slave ID + 0x50。
 */
class DmMotor : public MotorBase {
public:
    DmMotor();

    /**
     * @brief 配置一台达妙电机
     * @param id             电机 ID（slave ID）
     * @param bus            FDCAN 句柄
     * @param mode           默认控制模式
     * @param range          参数范围
     * @param tx_identifier  命令标识符基址，通常等于 id
     * @param rx_identifier  反馈标识符，通常为 id + 0x50
     * @param direction      安装方向，反馈与指令同时应用
     * @param reduction_ratio 协议轴到机构轴的外部减速比，默认 1
     * @return true 配置成功
     */
    bool init(uint8_t id, FDCAN_HandleTypeDef* bus, DmControlMode mode,
              const DmParamRange& range, uint32_t tx_identifier, uint32_t rx_identifier,
              int8_t direction = 1, float reduction_ratio = 1.0f);

    bool update(uint32_t identifier, const uint8_t* data, uint8_t length = 8U) override;
    void reset() override;
    uint32_t txIdentifier() const override { return tx_identifier_ + modeOffset(); }

    /** @brief 发送使能/失能/保存零点指令 */
    bool sendCommand(DmCommand command);

    /** @brief 发送 MIT 控制量 */
    bool sendMit(const DmControl& control);

    /** @brief 发送位置-速度控制量 */
    bool sendPositionVelocity(float position_rad, float velocity_rad_s);

    /** @brief 发送速度控制量 */
    bool sendVelocity(float velocity_rad_s);

    DmControlMode controlMode() const
    {
        return control_mode_;
    }

    const DmParamRange& paramRange() const
    {
        return range_;
    }

    /** @brief 一致性快照，包含同一时刻的反馈、计数和在线状态。 */
    MotorSnapshot<DmMotorData> readSnapshot() const { return snapshotOf(data_); }

    /** @brief 返回反馈副本；需要计数/在线状态时使用 readSnapshot()。 */
    DmMotorData data() const { return readSnapshot().data; }

private:
    bool faultKnown() const override
    {
        return data_.initialized && (data_.state <= 1U || data_.state >= 8U);
    }
    bool reportedFault() const override { return data_.initialized && data_.state >= 8U; }
    uint32_t modeOffset() const;
    static float uintToFloat(uint16_t value, float min_value, float max_value, uint8_t bits);
    static uint16_t floatToUint(float value, float min_value, float max_value, uint8_t bits);

    uint32_t tx_identifier_;
    DmControlMode control_mode_;
    DmParamRange range_;
    DmMotorData data_;
};

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
