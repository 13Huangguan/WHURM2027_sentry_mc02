/**
 * @file alg_motion_compensation.hpp
 * @brief 四轮全向底盘运动估计与目标速度补偿接口。
 * @copyright WHU-LuojiaFox (c) 2026
 *
 * 该模块根据四个全向轮的线速度和底盘坐标系下的 IMU 加速度，
 * 使用恒加速度模型卡尔曼滤波估计车体 vx/vy，并可按 Yaw 角速度生成
 * 一个可选的期望速度补偿量。
 *
 * @attention
 * 1. 模块只做算法计算，不直接读取电机、IMU 或 CAN；由 app_chassis 组装输入。
 * 2. 输入的轮速必须是逻辑轮序 1~4 的轮缘线速度，单位 m/s。
 * 3. 输入的加速度必须已经转换到底盘平面坐标系并去除重力，单位 m/s²。
 * 4. 轮子是否打滑由调用方判断；至少三个轮子有效时才进行轮速逆解。
 */

#pragma once

#include <cstdint>

namespace motion_compensation {

/** @brief 四轮全向底盘固定轮数。 */
constexpr uint8_t kWheelCount = 4U;

/** @brief 有效轮速位图全部有效时的掩码。 */
constexpr uint8_t kAllWheelsValidMask = 0x0FU;

/**
 * @brief 卡尔曼滤波与目标补偿参数。
 */
struct Config {
    /** @brief 加速度过程噪声谱密度，单位约为 m²/s⁵。 */
    float acceleration_process_noise;

    /** @brief 轮速逆解速度测量方差，(m/s)²。 */
    float wheel_measurement_variance;

    /** @brief IMU 加速度测量方差，(m/s²)²。 */
    float acceleration_measurement_variance;

    /** @brief 首次观测时的速度状态方差，(m/s)²。 */
    float initial_velocity_variance;

    /** @brief 首次观测时的加速度状态方差，(m/s²)²。 */
    float initial_acceleration_variance;

    /** @brief 文档中的补偿系数 K，最终 Kp = K * |yaw_rate|。 */
    float correction_gain_per_rad_s;

    /** @brief 补偿向量模长上限，m/s；传 0 表示不限制。 */
    float max_correction_mps;
};

/**
 * @brief 返回一组可用于初次调试的默认参数。
 * @note 默认参数没有经过当前实车标定，正式使用前必须根据日志调整。
 */
Config defaultConfig();

/**
 * @brief 一轮运动补偿测量输入。
 */
struct Measurement {
    /** @brief 逻辑轮序 1~4 的轮缘线速度，m/s。 */
    float wheel_linear_velocity_mps[kWheelCount] {};

    /**
     * @brief 有效轮位图。
     * bit0 对应 wheel_linear_velocity_mps[0]，依此类推。
     */
    uint8_t valid_wheel_mask = 0U;

    /**
     * @brief 期望坐标系相对底盘坐标系的 Yaw 角，rad。
     * @note 正负方向必须与文档公式及实车 IMU 安装方向一致。
     */
    float yaw_rad = 0.0f;

    /** @brief 本周期 Yaw 角速度，rad/s。 */
    float yaw_rate_rad_s = 0.0f;

    /** @brief 底盘坐标系 x 轴加速度，m/s²，已去除重力。 */
    float acceleration_x_mps2 = 0.0f;

    /** @brief 底盘坐标系 y 轴加速度，m/s²，已去除重力。 */
    float acceleration_y_mps2 = 0.0f;

    /** @brief acceleration_x/y 是否有效。 */
    bool acceleration_valid = false;

    /** @brief 本周期时长，s；必须大于 0。 */
    float dt_s = 0.0f;
};

/**
 * @brief 当前融合状态。
 */
struct Estimate {
    float vx_mps = 0.0f;             /**< 滤波后的 x 向速度 */
    float vy_mps = 0.0f;             /**< 滤波后的 y 向速度 */
    float ax_mps2 = 0.0f;            /**< 滤波后的 x 向加速度 */
    float ay_mps2 = 0.0f;            /**< 滤波后的 y 向加速度 */
    float yaw_rate_rad_s = 0.0f;     /**< 最近一次输入的 Yaw 角速度 */
    bool initialized = false;        /**< 是否已经有有效初始状态 */
    bool velocity_measurement_valid = false; /**< 本轮是否使用轮速测量 */
    uint8_t valid_wheel_mask = 0U;   /**< 本轮有效轮位图 */
};

/**
 * @brief 目标速度补偿结果。
 */
struct CorrectedVelocity {
    float vx_mps = 0.0f;  /**< 补偿后的 x 向目标速度 */
    float vy_mps = 0.0f;  /**< 补偿后的 y 向目标速度 */
    float correction_x_mps = 0.0f; /**< x 向补偿量 */
    float correction_y_mps = 0.0f; /**< y 向补偿量 */
    bool applied = false; /**< 是否实际应用了补偿 */
};

/**
 * @brief 四轮全向底盘运动补偿器。
 *
 * 调用流程：
 * 1. init() 配置参数。
 * 2. 每个控制周期准备 Measurement 并调用 update()。
 * 3. 用 estimate() 读取融合速度。
 * 4. 需要实验补偿时调用 compensate() 修改期望速度。
 *
 * 本类不使用动态内存，不作为线程安全对象使用。
 */
class OmniMotionCompensator {
public:
    /**
     * @brief 使用默认参数初始化。
     * @return 默认参数有效时返回 true。
     */
    bool init();

    /**
     * @brief 使用指定参数初始化。
     * @param config 滤波和目标补偿参数。
     * @return 参数有效时返回 true。
     */
    bool init(const Config& config);

    /**
     * @brief 清空滤波状态，保留当前配置。
     */
    void reset();

    /**
     * @brief 执行一次预测、轮速逆解和测量更新。
     * @param measurement 当前测量输入。
     * @return 输入有效且滤波状态已经建立时返回 true。
     * @note 少于三个有效轮时不会生成轮速速度测量，但若有 IMU
     *       加速度，滤波器仍可通过加速度预测状态。
     */
    bool update(const Measurement& measurement);

    /** @brief 获取最近一次融合状态。 */
    Estimate estimate() const;

    /**
     * @brief 按文档公式补偿目标速度。
     * @param target_vx_mps 期望 x 向速度，m/s。
     * @param target_vy_mps 期望 y 向速度，m/s。
     * @return 补偿后的目标速度及实际补偿量。
     * @note 公式为 target + K*|yaw_rate|*(target-estimate)。
     *       该补偿本质上是误差放大，默认未接入控制系统。
     */
    CorrectedVelocity compensate(float target_vx_mps, float target_vy_mps) const;

    /** @brief 查询是否已完成配置初始化。 */
    bool isConfigured() const;

    /** @brief 查询是否已经建立滤波状态。 */
    bool isInitialized() const;

    /** @brief 获取当前配置副本。 */
    Config config() const;

    /**
     * @brief 直接由四个轮速和一个 Yaw 角解算 vx/vy。
     * @param wheel_linear_velocity_mps 四轮轮缘线速度，m/s。
     * @param valid_mask 有效轮位图，至少需要三个有效轮。
     * @param yaw_rad 期望坐标系相对底盘坐标系的 Yaw 角，rad。
     * @param vx_mps 输出 x 向速度，m/s。
     * @param vy_mps 输出 y 向速度，m/s。
     * @return 解算成功时返回 true。
     * @note 该静态接口便于离线测试和标定，不改变滤波器内部状态。
     */
    static bool solveVelocity(
        const float wheel_linear_velocity_mps[kWheelCount],
        uint8_t valid_mask,
        float yaw_rad,
        float* vx_mps,
        float* vy_mps
    );

private:
    /**
     * @brief 单轴 [速度, 加速度] 两状态滤波器。
     */
    struct AxisFilter {
        float velocity = 0.0f;
        float acceleration = 0.0f;
        float p00 = 0.0f;
        float p01 = 0.0f;
        float p11 = 0.0f;
    };

    static bool configValid(const Config& config);
    static void resetAxis(
        AxisFilter* axis,
        float velocity,
        float acceleration,
        const Config& config
    );
    static void predictAxis(AxisFilter* axis, float dt_s, const Config& config);
    static void updateVelocity(AxisFilter* axis, float measured_velocity, float variance);
    static void updateAcceleration(AxisFilter* axis, float measured_acceleration, float variance);

    Config config_ {};
    AxisFilter vx_axis_ {};
    AxisFilter vy_axis_ {};
    float yaw_rate_rad_s_ = 0.0f;
    uint8_t valid_wheel_mask_ = 0U;
    bool configured_ = false;
    bool initialized_ = false;
    bool velocity_measurement_valid_ = false;
};

} // namespace motion_compensation

