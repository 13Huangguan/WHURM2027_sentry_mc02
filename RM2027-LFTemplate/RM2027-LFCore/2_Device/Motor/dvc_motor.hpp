/**
 * @file dvc_motor.hpp
 * @author LuojiaFox
 * @brief 电机公共基类与 CAN 接收分发
 * @version 3.0
 * @date 2026-10-20
 *
 * @attention 本文件只放公共部分；具体厂商驱动见：
 *            - dvc_dji_motor.hpp
 *            - dvc_dm_motor.hpp
 *            - dvc_lk_motor.hpp
 *            新增电机时继承 MotorBase，实现 update()/reset()，
 *            再调用 MotorBase::registerMotor() 注册。
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

#include <stdint.h>
#include <math.h>

/** @brief 保存并恢复中断屏蔽状态；仅用于短数据复制，禁止在作用域内发送 CAN。 */
class MotorCriticalSection {
public:
    MotorCriticalSection() : mask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~MotorCriticalSection() { __DMB(); __set_PRIMASK(mask_); }
    MotorCriticalSection(const MotorCriticalSection&) = delete;
    MotorCriticalSection& operator=(const MotorCriticalSection&) = delete;
private:
    uint32_t mask_;
};

template <typename Data>
struct MotorSnapshot {
    Data data;
    uint32_t last_rx_tick;
    uint32_t rx_count;
    bool online;
    bool fault_known;
    bool fault;
};

/* Exported macros -----------------------------------------------------------*/

/** @brief 默认电机离线判定时间，单位 ms */
constexpr uint32_t kDefaultMotorOfflineTimeoutMs = 100U;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机类型
 * @note 新增厂商电机时在此追加。
 */
enum class MotorType : uint8_t {
    Unknown = 0, /*!< 未知类型 */
    DjiGm6020,   /*!< DJI GM6020，电压控制 */
    DjiM3508,    /*!< DJI M3508，电流控制 */
    DjiM2006,    /*!< DJI M2006，电流控制 */
    Dm,          /*!< 达妙电机 */
    Lk,          /*!< 瓴控电机（MF/MG 系列） */
};

/**
 * @brief 电机公共接口
 * @note 只管“反馈解析”和“在线状态”，控制量由各子类自行定义。
 *       update() 会在 CAN 接收中断中调用，函数内不要做阻塞操作。
 */
class MotorBase {
public:
    static constexpr uint8_t kMaxMotors = 24U;

    MotorBase(MotorType type, FDCAN_HandleTypeDef* bus, uint8_t id, uint32_t rx_identifier);
    MotorBase(const MotorBase&) = delete;
    MotorBase& operator=(const MotorBase&) = delete;

    /**
     * @brief 用一帧 CAN 数据刷新电机状态
     * @param identifier 标准帧标识符
     * @param data       8 字节数据
     * @return true 表示该帧属于本电机且已解析
     */
    virtual bool update(uint32_t identifier, const uint8_t* data, uint8_t length = 8U) = 0;

    /** @brief 清空反馈数据与离线计时 */
    virtual void reset() = 0;

    MotorType type() const
    {
        return type_;
    }

    uint8_t id() const
    {
        return id_;
    }

    FDCAN_HandleTypeDef* bus() const
    {
        return bus_;
    }

    uint32_t rxIdentifier() const
    {
        return rx_identifier_;
    }

    /** @brief 参与统一发送时的帧标识符；返回 0 表示不参与 */
    virtual uint32_t txIdentifier() const
    {
        return 0U;
    }

    /** @brief 在发送帧中的槽位，0~3 */
    virtual uint8_t txSlot() const
    {
        return 0U;
    }

    virtual bool sharesTxFrame() const { return false; }

    int8_t direction() const
    {
        return direction_;
    }

    /**
     * @brief 设置安装方向
     * @param direction 正数表示与机构定义方向一致，负数表示相反
     */
    bool setDirection(int8_t direction)
    {
        if (isRegistered()) { return false; }
        direction_ = (direction < 0) ? -1 : 1;
        return true;
    }

    float reductionRatio() const
    {
        return reduction_ratio_;
    }

    /**
     * @brief 设置输出轴减速比
     * @param ratio 例如 M3508 为 19，GM6020 为 1；仅初始化阶段设置；无效值返回 false 并保留原配置
     */
    bool setReductionRatio(float ratio)
    {
        if (isRegistered() || !isfinite(ratio) || ratio < 1.0e-6f || ratio > 1.0e6f) {
            return false;
        }
        reduction_ratio_ = ratio;
        return true;
    }

    void setOfflineTimeout(uint32_t timeout_ms)
    {
        MotorCriticalSection lock;
        offline_timeout_ms_ = timeout_ms;
    }

    uint32_t lastRxTick() const
    {
        MotorCriticalSection lock;
        return last_rx_tick_;
    }

    /** @brief 已收到的反馈帧数，可用于判断数据是否刷新 */
    uint32_t rxCount() const
    {
        MotorCriticalSection lock;
        return rx_count_;
    }

    /** @brief 按给定时刻判断电机是否在线 */
    bool isOnline(uint32_t now_ms) const;

    /** @brief 按当前 HAL 节拍判断电机是否在线 */
    bool isOnline() const;

    /* ---- 电机注册与 CAN 接收分发，无需额外的 manager 类 ---- */

    /** @brief 注册电机，重复注册或标识符冲突时返回 false */
    static bool registerMotor(MotorBase& motor);

    /**
     * @brief 在 CAN 接收回调中分发一帧数据
     * @note  按 (CAN 口, 接收标识符) 找到电机并调用其 update()
     */
    static void onCanFrame(FDCAN_HandleTypeDef* bus, uint32_t identifier,
                           const uint8_t* data, uint8_t length);

    /** @brief 注册后禁止再次 init；电机对象必须具有静态生命周期。 */
    bool isRegistered() const;

    /** @brief 按注册顺序访问电机，越界返回 nullptr */
    static MotorBase* motorAt(uint8_t index);

    static uint8_t motorCount();

    /** @brief 所有已注册电机是否都在线，未注册任何电机时返回 true */
    static bool allOnline(uint32_t now_ms);

protected:
    ~MotorBase() = default;

    /** @brief 未携带故障位的反馈不能用在线状态推断无故障。 */
    virtual bool faultKnown() const { return false; }
    virtual bool reportedFault() const { return false; }

    void configure(MotorType type, FDCAN_HandleTypeDef* bus, uint8_t id, uint32_t rx_identifier);

    /** @brief 更新最近一次反馈时刻 */
    void refreshRxTick();

    /** @brief 清空离线计时，使电机回到“从未收到反馈”状态 */
    void clearRxTick()
    {
        last_rx_tick_ = 0U;
        rx_count_ = 0U;
        received_ = false;
    }

    template <typename Data>
    MotorSnapshot<Data> snapshotOf(const Data& data) const
    {
        MotorCriticalSection lock;
        return {data, last_rx_tick_, rx_count_, isOnline(HAL_GetTick()),
                faultKnown(), reportedFault()};
    }

private:
    MotorType type_;
    FDCAN_HandleTypeDef* bus_;
    uint8_t id_;
    uint32_t rx_identifier_;
    int8_t direction_;
    float reduction_ratio_;
    uint32_t last_rx_tick_;
    uint32_t rx_count_;
    uint32_t offline_timeout_ms_;
    bool received_ = false;

    /* 静态注册表 */
    static MotorBase* motors_[kMaxMotors];
    static volatile uint8_t motor_count_;
};

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
