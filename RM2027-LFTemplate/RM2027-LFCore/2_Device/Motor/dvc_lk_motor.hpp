/**
 * @file dvc_lk_motor.hpp
 * @author LuojiaFox
 * @brief 瓴控电机驱动（MF/MG 系列）
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
 * @brief 瓴控电机反馈数据
 * @note 驱动器可能用 0x9C 上报状态，也可能在控制命令的回复里回显命令字
 *       （例如发 0xA2 速度控制，反馈帧的 data[0] 也是 0xA2），
 *       两种帧的数据布局相同。
 */
struct LkMotorData {
    int8_t temperature = 0;      /*!< 温度，摄氏度 */
    int16_t iq_raw = 0;          /*!< 电流原始值 */
    int16_t speed_dps = 0;       /*!< 输出轴速度，deg/s */
    uint16_t encoder = 0;        /*!< 单圈编码器原始值 */
    float current_a = 0.0f;      /*!< 电流，A */
    float velocity_rad_s = 0.0f; /*!< 输出轴角速度，rad/s */
    float angle_rad = 0.0f;      /*!< 单圈角度，-pi~pi */
    uint8_t frame_type = 0;      /*!< 最近一帧 data[0]：0x9C 或控制命令回显 */
    bool initialized = false;    /*!< 是否收到过反馈 */
    uint8_t raw[8] = {};         /*!< 最近一帧原始反馈数据 */
};

/**
 * @brief 瓴控电机（MF/MG 系列）
 *
 * 命令与反馈使用同一个标准帧标识符：0x140 + ID（ID 1~32）。
 * 常用命令：
 * | 命令 | 含义 |
 * | --- | --- |
 * | 0x80 | 失能 |
 * | 0x81 | 停止 |
 * | 0x88 | 使能 |
 * | 0xA1 | 转矩电流控制 |
 * | 0xA2 | 速度控制（0.01 dps/LSB） |
 * | 0xA3 | 位置控制（0.01 deg/LSB） |
 * | 0xA4 | 位置控制 + 最大速度 |
 */
class LkMotor : public MotorBase {
public:
    LkMotor();

    /**
     * @brief 配置一台瓴控电机
     * @param id              电机 ID，1~32
     * @param bus             FDCAN 句柄
     * @param encoder_bits    编码器位数，14/15/16，默认 16
     * @param mg_current      true 使用 MG 系列电流比例（66/4096 A/LSB），
     *                        false 使用 MF 系列（33/4096 A/LSB）
     * @param reduction_ratio 协议轴到机构轴的外部减速比，默认 1
     * @param direction       安装方向，反馈和指令同时应用
     * @return true 配置成功
     */
    bool init(uint8_t id, FDCAN_HandleTypeDef* bus, uint8_t encoder_bits = 16U,
              bool mg_current = false, float reduction_ratio = 1.0f, int8_t direction = 1);

    bool update(uint32_t identifier, const uint8_t* data, uint8_t length = 8U) override;
    void reset() override;
    uint32_t txIdentifier() const override { return commandIdentifier(); }

    /** @brief 使能 / 失能 / 停止 */
    bool sendEnable();
    bool sendDisable();
    bool sendStop();

    /** @brief 请求一帧 0x9C 状态反馈（驱动器不主动上报时使用） */
    bool sendReadStatus();

    /** @brief 转矩电流控制，原始值 -2048~2048 */
    bool sendTorqueCurrent(int16_t iq);

    /** @brief 电流控制，单位 A，内部换算为原始电流量 */
    bool sendCurrent(float current_a);

    /**
     * @brief 速度控制
     * @param velocity_rad_s 输出轴目标角速度，rad/s
     * @param current_limit  最大转矩电流幅值，取绝对值后限幅到 2048；0 使用驱动器默认值
     */
    bool sendSpeed(float velocity_rad_s, int16_t current_limit = 0);

    /**
     * @brief 位置控制
     * @param angle_rad      机构轴目标角度，换算后在协议轴限幅 ±360°
     * @param max_speed_dps  最大速度，deg/s；传 0 使用不带最大速度字段的 0xA3
     */
    bool sendPosition(float angle_rad, uint16_t max_speed_dps = 0);

    uint32_t commandIdentifier() const
    {
        return 0x140U + (uint32_t)id();
    }

    /** @brief 一致性快照，包含同一时刻的反馈、计数和在线状态。 */
    MotorSnapshot<LkMotorData> readSnapshot() const { return snapshotOf(data_); }

    /** @brief 返回反馈副本；需要计数/在线状态时使用 readSnapshot()。 */
    LkMotorData data() const { return readSnapshot().data; }

private:
    bool sendCommand(uint8_t command, uint8_t offset, const uint8_t* payload, uint8_t length);
    static int16_t clampIq(int32_t value);

    uint8_t encoder_bits_;
    float current_per_lsb_;
    LkMotorData data_;
};

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
