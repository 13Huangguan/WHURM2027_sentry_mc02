/**
 * @file alg_slip_compensation2.hpp
 * @brief 基于执行器零空间的轮间打滑负载重分配。
 */

#ifndef ALG_SLIP_REDISTRIBUTION_HPP
#define ALG_SLIP_REDISTRIBUTION_HPP

#include <cstdint>

/** @brief 单次求解支持的最大执行器数量。 */
#define ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX 8

namespace slip_redistribution {

/** @brief 打滑重分配状态。 */
enum class Status : uint8_t {
    Disabled = 0,
    Ready,
    NormalSlip,
    AbnormalSlip,
    Recovering,
    Limited,
    InvalidInput,
};

/**
 * @brief 打滑重分配参数。
 * @note 默认参数仅用于初始调试，必须根据实车轮速和电流日志重新标定。
 */
struct Config {
    /** @brief 是否启用重分配；关闭时输出等于基础命令。 */
    bool enabled;

    /** @brief 控制周期，s。 */
    float dt_s;

    /** @brief 目标速度低于该值时不做轮速比值判断，rad/s。 */
    float min_tracking_speed_rad_s;

    /** @brief 轮速比一阶低通中的旧值权重。 */
    float speed_ratio_filter_alpha;

    /** @brief 正常打滑进入阈值。 */
    float normal_ratio_enter;

    /** @brief 正常打滑退出阈值。 */
    float normal_ratio_exit;

    /** @brief 最高与最低轮速比之差达到该值时进入正常打滑判断。 */
    float normal_ratio_spread;

    /** @brief 异常打滑进入阈值。 */
    float abnormal_ratio_enter;

    /** @brief 异常打滑退出阈值。 */
    float abnormal_ratio_exit;

    /** @brief 异常轮速比与最低轮速比的最小差值。 */
    float abnormal_ratio_spread;

    /** @brief 选择增载轮时要求的最低速度估计置信度。 */
    float load_confidence_min;

    /** @brief 零空间系数相对基础命令范数的最大比例。 */
    float max_adjustment_fraction;

    /** @brief 进入打滑重分配的零空间系数变化率，1/s。 */
    float adjustment_rate_per_s;

    /** @brief 退出打滑时的零空间系数恢复率，1/s。 */
    float recovery_rate_per_s;
};

/** @brief 返回一组保守的初始参数。 */
Config defaultConfig();

/**
 * @brief 基于零空间方向的轮间打滑负载重分配器。
 *
 * 输入的 base_command 与 output_command 必须使用同一单位和方向约定。
 * 当前底盘传入四轮目标电流。nullspace 必须是与基础分配矩阵对应的
 * 一维零空间方向，模块会将其归一化并保持：
 *
 * output = base + alpha * ||base|| * nullspace
 *
 * 因此理想情况下，重分配不会改变整车广义力。
 */
class SlipRedistributor {
public:
    /**
     * @brief 初始化重分配器。
     * @param config 控制参数。
     * @param nullspace 零空间方向数组。
     * @param actuator_count 执行器数量，范围 2~8。
     * @return 参数有效时返回 true。
     */
    bool init(
        const Config& config,
        const float* nullspace,
        uint8_t actuator_count
    );

    /**
     * @brief 执行一次打滑检测和零空间重分配。
     * @param base_command 基础执行器命令。
     * @param actual_speed 实际输出轴速度，rad/s。
     * @param target_speed 目标输出轴速度，rad/s。
     * @param confidence 可选的逐轮速度估计置信度数组，可传 nullptr。
     * @param slip_wheel_mask 由运动估计器给出的打滑轮位图。
     * @param slip_detection_valid 打滑位图是否有效。
     * @param command_limit 单执行器命令绝对上限。
     * @param output_command 重分配后的执行器命令。
     * @return 输入有效时返回 true；非法输入时输出清零并返回 false。
     */
    bool update(
        const float* base_command,
        const float* actual_speed,
        const float* target_speed,
        const float* confidence,
        uint8_t slip_wheel_mask,
        bool slip_detection_valid,
        float command_limit,
        float* output_command
    );

    /** @brief 清空零空间系数、滤波器和诊断状态，保留配置。 */
    void reset();

    /** @brief 查询最近一次求解状态。 */
    Status status() const {
        return this->status_;
    }

    /** @brief 查询当前零空间系数。 */
    float coefficient() const {
        return this->coefficient_;
    }

    /** @brief 查询本周期被选为增载对象的轮位图。 */
    uint8_t loadedWheelMask() const {
        return this->loaded_wheel_mask_;
    }

    /** @brief 查询本周期被判为异常高速打滑的轮位图。 */
    uint8_t abnormalWheelMask() const {
        return this->abnormal_wheel_mask_;
    }

    /** @brief 查询因边界限制没有达到目标系数的轮位图。 */
    uint8_t limitedWheelMask() const {
        return this->limited_wheel_mask_;
    }

    /**
     * @brief 查询第 index 个执行器的滤波后轮速比。
     * @return 比值无效时返回 0。
     */
    float speedRatio(uint8_t index) const {
        return index < this->actuator_count_ ? this->speed_ratio_[index] : 0.0f;
    }

private:
    /** @brief 清空逐轮轮速比和诊断位图。 */
    void resetWheelState();

    /**
     * @brief 计算满足命令限幅和原方向约束的零空间系数区间。
     * @param base_command 基础命令。
     * @param command_limit 命令绝对上限。
     * @param lower 输出区间下界。
     * @param upper 输出区间上界。
     * @return 区间有效时返回 true。
     */
    bool feasibleRange(
        const float* base_command,
        float command_limit,
        float* lower,
        float* upper
    ) const;

    Config config_ {};
    uint8_t actuator_count_ = 0U;
    float nullspace_[ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX] {};
    float speed_ratio_[ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX] {};
    bool ratio_ready_[ALG_SLIP_REDISTRIBUTION_ACTUATOR_MAX] {};
    float coefficient_ = 0.0f;
    uint8_t loaded_wheel_mask_ = 0U;
    uint8_t abnormal_wheel_mask_ = 0U;
    uint8_t limited_wheel_mask_ = 0U;
    bool normal_active_ = false;
    bool abnormal_active_ = false;
    Status status_ = Status::Disabled;
};

} // namespace slip_redistribution

#endif // ALG_SLIP_REDISTRIBUTION_HPP
