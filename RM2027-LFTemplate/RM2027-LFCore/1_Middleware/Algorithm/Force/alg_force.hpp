/**
 * @file alg_force.hpp
 * @brief 多执行器力分配与执行器模型补偿接口。
 * @copyright WHU-LuojiaFox (c) 2026
 *
 * 该模块将上层给出的广义力目标分配到多个执行器，并叠加惯性、摩擦、
 * 前馈和重力补偿。所有位置、速度、力矩和机械参数均使用执行器输出轴坐标。
 */

#ifndef ALG_FORCE_HPP
#define ALG_FORCE_HPP

#include <cstdint>

/** @brief 单次求解支持的最大执行器数量。 */
#define ALG_FORCE_ACTUATOR_MAX 8

/** @brief 单次求解支持的最大广义力自由度。 */
#define ALG_FORCE_DOF_MAX 4

/**
 * @brief 单个执行器的机械与传动参数。
 * @note 所有机械量均位于执行器输出轴；参数由 ForceSolver::init() 复制保存。
 */
struct ForceActuatorParam {
    float torque_constant;     /**< 输出轴力矩常数，N*m/A */
    float reduction_ratio;     /**< 电机到执行器输出轴的减速比 */
    int8_t direction;          /**< 输出轴方向，+1 或 -1 */
    float torque_max;          /**< 输出轴力矩绝对上限，N*m */
    float inertia;             /**< 执行器等效转动惯量，kg*m^2 */
    float viscous_friction;    /**< 粘性摩擦系数，N*m*s/rad */
    float coulomb_friction;    /**< 库仑摩擦力矩幅值，N*m */
    float gravity_amplitude;   /**< 重力补偿力矩幅值，N*m；0 表示关闭 */
    float gravity_offset;      /**< 重力补偿相位偏移，rad */
};

/**
 * @brief 多执行器力分配与执行器补偿求解器。
 *
 * 典型调用顺序：
 * 1. init() 配置执行器数量、自由度、控制周期和机械参数。
 * 2. setDistribution() 设置执行器到广义力的分配矩阵。
 * 3. 每个控制周期调用 setTarget()、setNow() 和 setEnable(true)。
 * 4. 调用 timCalculatePeriodElapsedCallback() 更新输出。
 * 5. 通过 getTorque() 读取补偿后力矩，或通过 getOut() 读取驱动控制量。
 *
 * 本类只提供映射和补偿计算，不发送 CAN，也不直接操作电机。
 */
class ForceSolver {
public:
    /**
     * @brief 初始化求解器并复制执行器参数。
     * @param param 执行器参数数组，不能为空。
     * @param actuator_num 有效执行器数量，最大 ALG_FORCE_ACTUATOR_MAX。
     * @param dof_num 广义力自由度，范围 1~ALG_FORCE_DOF_MAX。
     * @param dt 控制周期，s；非正数或非法值按 0.001 s 处理。
     * @note 初始化会清空此前设置的目标、反馈和输出。
     *       分配矩阵未设置时，默认使用单位对角矩阵。
     */
    void init(
        const ForceActuatorParam* param,
        uint8_t actuator_num,
        uint8_t dof_num,
        float dt
    );

    /**
     * @brief 设置执行器到广义力的分配矩阵。
     * @param matrix 按行优先排列的 actuator_num x dof_num 矩阵数据。
     * @note 求解器会复制矩阵，调用方无需保持 matrix 生命周期。
     */
    void setDistribution(const float* matrix);

    /**
     * @brief 设置本周期期望的广义力目标。
     * @param generalized_force 长度为 dof_num 的目标数组。
     */
    void setTarget(const float* generalized_force);

    /**
     * @brief 更新执行器位置和速度反馈。
     * @param position 长度为 actuator_num 的位置数组，传 nullptr 时保持旧值。
     * @param velocity 长度为 actuator_num 的速度数组，传 nullptr 时保持旧值。
     * @note 传入的速度会经过一阶低通滤波。
     */
    void setNow(const float* position, const float* velocity);

    /**
     * @brief 设置各执行器的附加前馈力矩。
     * @param torque 长度为 actuator_num 的前馈力矩数组，N*m。
     */
    void setFeedforwardTorque(const float* torque);

    /**
     * @brief 设置速度一阶低通滤波系数。
     * @param alpha 范围 0~1；越大越平滑，较小值更跟随新反馈。
     */
    void setVelocityFilterAlpha(float alpha);

    /**
     * @brief 启用或停用求解器。
     * @param enable true 启用输出；false 清空目标、反馈和输出。
     */
    void setEnable(bool enable);

    /**
     * @brief 执行一次力分配和补偿计算。
     * @note 应在控制任务中按 init() 给出的周期调用。
     *       未启用、没有执行器或矩阵计算失败时 torque/out 输出为零。
     */
    void timCalculatePeriodElapsedCallback();

    /**
     * @brief 清空目标、反馈、滤波器状态和输出。
     * @note 不会清除 init() 配置的执行器参数和分配矩阵。
     */
    void reset();

    /**
     * @brief 构建四轮全向底盘的 4x3 分配矩阵。
     * @param wheel_radius 轮半径，m。
     * @param wheel_center_distance 轮中心到车体中心的距离，m。
     * @param matrix 输出缓冲区，至少包含 4*3 个 float。
     * @return 参数有效时返回 true。
     * @note 矩阵按执行器索引行优先存储，广义力顺序为 [Fx, Fy, Mz]。
     */
    static bool
    buildOmniDistribution(float wheel_radius, float wheel_center_distance, float* matrix);

    /** @brief 查询求解器是否启用。 */
    bool isEnable() const {
        return this->enabled;
    }

    /** @brief 获取有效执行器数量。 */
    uint8_t getActuatorNum() const {
        return this->actuator_num;
    }

    /** @brief 获取广义力自由度。 */
    uint8_t getDofNum() const {
        return this->dof_num;
    }

    /**
     * @brief 获取指定执行器补偿后的输出轴力矩。
     * @param index 执行器索引，越界时返回 0。
     */
    float getTorque(uint8_t index) const {
        return index < this->actuator_num ? this->torque[index] : 0.0f;
    }

    /**
     * @brief 获取指定执行器的最终驱动控制量。
     * @param index 执行器索引，越界时返回 0。
     * @note 单位由 torque_constant 和 reduction_ratio 决定，通常为 A。
     */
    float getOut(uint8_t index) const {
        return index < this->actuator_num ? this->out[index] : 0.0f;
    }

    /**
     * @brief 获取指定执行器滤波后的速度。
     * @param index 执行器索引，越界时返回 0。
     */
    float getVelocity(uint8_t index) const {
        return index < this->actuator_num ? this->velocity[index] : 0.0f;
    }

    /**
     * @brief 获取指定自由度的广义力目标。
     * @param index 自由度索引，越界时返回 0。
     */
    float getGeneralizedForce(uint8_t index) const {
        return index < this->dof_num ? this->target_generalized_force[index] : 0.0f;
    }

private:
    uint8_t actuator_num = 0;        /**< 有效执行器数量 */
    uint8_t dof_num = 0;             /**< 有效广义力自由度 */
    bool enabled = false;            /**< 求解与输出使能 */
    float inverse_dt = 1000.0f;      /**< 控制周期倒数，1/s */
    float velocity_filter_alpha = 0.9f;              /**< 速度滤波旧值权重 */
    float velocity_filter_weight = 1.0f - 0.9f;      /**< 速度滤波新值权重 */

    /** @brief 初始化时复制的执行器参数。 */
    ForceActuatorParam param[ALG_FORCE_ACTUATOR_MAX] {};

    /** @brief 紧凑行优先分配矩阵，每行仅保存 dof_num 个元素。 */
    float distribution[ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX] {};

    float target_generalized_force[ALG_FORCE_DOF_MAX] {}; /**< 广义力目标 */
    float feedforward_torque[ALG_FORCE_ACTUATOR_MAX] {};  /**< 执行器前馈力矩 */
    float position[ALG_FORCE_ACTUATOR_MAX] {};            /**< 执行器位置反馈 */
    float velocity[ALG_FORCE_ACTUATOR_MAX] {};            /**< 滤波后速度反馈 */
    float pre_velocity[ALG_FORCE_ACTUATOR_MAX] {};         /**< 上一周期滤波速度 */
    bool velocity_filter_ready[ALG_FORCE_ACTUATOR_MAX] {}; /**< 滤波器首帧标志 */
    float torque[ALG_FORCE_ACTUATOR_MAX] {};               /**< 补偿后输出轴力矩 */
    float out[ALG_FORCE_ACTUATOR_MAX] {};                  /**< 最终驱动控制量 */

    /** @brief 力矩到驱动控制量的换算系数。 */
    float output_scale[ALG_FORCE_ACTUATOR_MAX] {};

    /** @brief 由等效惯量换算的加速度补偿系数。 */
    float acceleration_gain[ALG_FORCE_ACTUATOR_MAX] {};

    /** @brief 每个执行器的力矩绝对值上限。 */
    float torque_limit[ALG_FORCE_ACTUATOR_MAX] {};
};

#endif // ALG_FORCE_HPP
