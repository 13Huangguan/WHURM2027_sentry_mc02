// Force allocation and actuator compensation. Copyright WHU-LuojiaFox (c) 2026.
#ifndef ALG_FORCE_HPP
#define ALG_FORCE_HPP

#include <cstdint>

#define ALG_FORCE_ACTUATOR_MAX 8
#define ALG_FORCE_DOF_MAX 4

// All mechanical quantities refer to the actuator output shaft.
struct ForceActuatorParam {
    float torque_constant; // N*m/A.
    float reduction_ratio;
    int8_t direction; // +1 or -1.
    float torque_max; // N*m.
    float inertia; // kg*m^2.
    float viscous_friction;
    float coulomb_friction;
    float gravity_amplitude; // N*m, zero disables gravity compensation.
    float gravity_offset; // rad.
};

class ForceSolver {
public:
    void init(const ForceActuatorParam* param, uint8_t actuator_num, uint8_t dof_num, float dt);
    // Row-major actuator_num x dof_num. The solver copies the supplied matrix.
    void setDistribution(const float* matrix);
    void setTarget(const float* generalized_force);
    void setNow(const float* position, const float* velocity);
    void setFeedforwardTorque(const float* torque);
    void setVelocityFilterAlpha(float alpha);
    void setEnable(bool enable);
    void timCalculatePeriodElapsedCallback();
    void reset();

    // Minimum-norm force allocation for a four-wheel omni chassis (4 x 3).
    static bool
    buildOmniDistribution(float wheel_radius, float wheel_center_distance, float* matrix);

    bool isEnable() const {
        return this->enabled;
    }
    uint8_t getActuatorNum() const {
        return this->actuator_num;
    }
    uint8_t getDofNum() const {
        return this->dof_num;
    }
    float getTorque(uint8_t index) const {
        return index < this->actuator_num ? this->torque[index] : 0.0f;
    }
    float getOut(uint8_t index) const {
        return index < this->actuator_num ? this->out[index] : 0.0f;
    }
    float getVelocity(uint8_t index) const {
        return index < this->actuator_num ? this->velocity[index] : 0.0f;
    }
    float getGeneralizedForce(uint8_t index) const {
        return index < this->dof_num ? this->target_generalized_force[index] : 0.0f;
    }

private:
    uint8_t actuator_num = 0;
    uint8_t dof_num = 0;
    bool enabled = false;
    float inverse_dt = 1000.0f;
    float velocity_filter_alpha = 0.9f;
    float velocity_filter_weight = 1.0f - 0.9f;
    ForceActuatorParam param[ALG_FORCE_ACTUATOR_MAX] {};
    // Compact active matrix: each row has dof_num elements, without padding.
    float distribution[ALG_FORCE_ACTUATOR_MAX * ALG_FORCE_DOF_MAX] {};
    float target_generalized_force[ALG_FORCE_DOF_MAX] {};
    float feedforward_torque[ALG_FORCE_ACTUATOR_MAX] {};
    float position[ALG_FORCE_ACTUATOR_MAX] {};
    float velocity[ALG_FORCE_ACTUATOR_MAX] {};
    float pre_velocity[ALG_FORCE_ACTUATOR_MAX] {};
    bool velocity_filter_ready[ALG_FORCE_ACTUATOR_MAX] {};
    float torque[ALG_FORCE_ACTUATOR_MAX] {};
    float out[ALG_FORCE_ACTUATOR_MAX] {};
    float output_scale[ALG_FORCE_ACTUATOR_MAX] {};
    float acceleration_gain[ALG_FORCE_ACTUATOR_MAX] {};
    float torque_limit[ALG_FORCE_ACTUATOR_MAX] {};
};

#endif // ALG_FORCE_HPP
