/**
 * @file alg_force.cpp
 * @author WHU-LuojiaFox
 * @brief 力控解算实现
 * @version 1.0
 * @date 2026-10-13 1.0 首版
 *
 * @copyright WHU-LuojiaFox (c) 2026
 *
 * PC 侧自检（不需要任何 ARM 环境）：
 *     g++ -std=c++14 -DALG_FORCE_HOST_TEST alg_force.cpp -o alg_force_test
 *     ./alg_force_test
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_force.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 符号函数，返回 -1 / 0 / +1
 *
 * @param value 输入
 * @return float 符号
 */
static float forceSolverSign(float value) {
    if (value > 0.0f) {
        return (1.0f);
    }
    if (value < 0.0f) {
        return (-1.0f);
    }
    return (0.0f);
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 限幅
 *
 * @param value    输入
 * @param minValue 下限
 * @param maxValue 上限
 * @return float 限幅后的值
 */
float ForceSolver::constrainValue(float value, float minValue, float maxValue) {
    if (value > maxValue) {
        return (maxValue);
    }
    if (value < minValue) {
        return (minValue);
    }
    return (value);
}

/**
 * @brief 初始化解算器
 *
 * @param param        执行器参数数组
 * @param actuatorNum  执行器数量
 * @param dofNum       广义力维数
 * @param dt           解算周期, s
 */
void ForceSolver::init(
    const ForceActuatorParam_t *param,
    uint8_t actuatorNum,
    uint8_t dofNum,
    float dt
) {
    if (actuatorNum > ALG_FORCE_ACTUATOR_MAX) {
        actuatorNum = ALG_FORCE_ACTUATOR_MAX;
    }
    if (dofNum > ALG_FORCE_DOF_MAX) {
        dofNum = ALG_FORCE_DOF_MAX;
    }
    if (dofNum == 0U) {
        dofNum = 1U;
    }

    ActuatorNum = actuatorNum;
    DofNum = dofNum;
    Dt = (dt > 0.0f) ? dt : 0.001f;

    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        Param[i] = param[i];
    }

    // 默认分配矩阵取单位阵：第 i 维广义力只驱动第 i 个执行器
    for (uint8_t i = 0U; i < ALG_FORCE_ACTUATOR_MAX; ++i) {
        for (uint8_t j = 0U; j < ALG_FORCE_DOF_MAX; ++j) {
            Distribution[i][j] = 0.0f;
        }
    }

    const uint8_t diagonalNum = (ActuatorNum < DofNum) ? ActuatorNum : DofNum;
    for (uint8_t i = 0U; i < diagonalNum; ++i) {
        Distribution[i][i] = 1.0f;
    }

    reset();

    Status = ForceSolverStatus_Disable;
}

/**
 * @brief 设置分配矩阵
 *
 * @param matrix 行优先矩阵首地址，尺寸 actuatorNum x dofNum
 */
void ForceSolver::setDistribution(const float *matrix) {
    if (matrix == nullptr) {
        return;
    }

    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        for (uint8_t j = 0U; j < DofNum; ++j) {
            Distribution[i][j] = matrix[i * DofNum + j];
        }
    }
}

/**
 * @brief 设置期望广义力
 *
 * @param generalizedForce 期望力数组，长度 dofNum
 */
void ForceSolver::setTarget(const float *generalizedForce) {
    if (generalizedForce == nullptr) {
        return;
    }

    for (uint8_t j = 0U; j < DofNum; ++j) {
        TargetGeneralizedForce[j] = generalizedForce[j];
    }
}

/**
 * @brief 设置本轮反馈
 *
 * @param position 位置数组，长度 actuatorNum
 * @param velocity 速度数组，长度 actuatorNum
 */
void ForceSolver::setNow(const float *position, const float *velocity) {
    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        if (position != nullptr) {
            Position[i] = position[i];
        }

        if (velocity != nullptr) {
            // 第一帧直接用输入初始化，避免从 0 慢慢爬上来
            if (!VelocityFilterReady[i]) {
                VelocityFilterOutput[i] = velocity[i];
                VelocityFilterReady[i] = true;
            }

            VelocityFilterOutput[i] = VelocityFilterAlpha * VelocityFilterOutput[i] +
                                      (1.0f - VelocityFilterAlpha) * velocity[i];
            Velocity[i] = VelocityFilterOutput[i];
        }
    }
}

/**
 * @brief 设置额外前馈力矩
 *
 * @param torque 前馈力矩数组，长度 actuatorNum
 */
void ForceSolver::setFeedforwardTorque(const float *torque) {
    if (torque == nullptr) {
        return;
    }

    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        FeedforwardTorque[i] = torque[i];
    }
}

/**
 * @brief 设置速度反馈的低通滤波系数
 *
 * @param alpha 滤波系数 [0, 1]
 */
void ForceSolver::setVelocityFilterAlpha(float alpha) {
    VelocityFilterAlpha = constrainValue(alpha, 0.0f, 1.0f);
}

/**
 * @brief 使能/失能
 *
 * @param enable true 使能
 */
void ForceSolver::setEnable(bool enable) {
    Status = enable ? ForceSolverStatus_Enable : ForceSolverStatus_Disable;

    if (!enable) {
        reset();
    }
}

/**
 * @brief 周期解算
 */
void ForceSolver::timCalculatePeriodElapsedCallback() {
    if (Status == ForceSolverStatus_Disable) {
        for (uint8_t i = 0U; i < ActuatorNum; ++i) {
            Torque[i] = 0.0f;
            Out[i] = 0.0f;
            PreVelocity[i] = Velocity[i];
        }
        return;
    }

    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        // 1. 分配解算：期望广义力 -> 本执行器的期望力矩
        float torque = 0.0f;
        for (uint8_t j = 0U; j < DofNum; ++j) {
            torque += Distribution[i][j] * TargetGeneralizedForce[j];
        }

        // 2. 动力学补偿：惯量前馈 + 粘滞摩擦 + 库仑摩擦
        const float acceleration = (Velocity[i] - PreVelocity[i]) / Dt;
        torque += Param[i].Inertia * acceleration;
        torque += Param[i].ViscousFriction * Velocity[i];
        torque += Param[i].CoulombFriction * forceSolverSign(Velocity[i]);

        // 3. 重力矩补偿：A * cos(theta - phi)
        torque += Param[i].GravityAmplitude * cosf(Position[i] - Param[i].GravityOffset);

        // 4. 叠加额外前馈并限幅
        torque += FeedforwardTorque[i];
        const float torqueMax = fabsf(Param[i].TorqueMax);
        torque = constrainValue(torque, -torqueMax, torqueMax);
        Torque[i] = torque;

        // 5. 力矩 -> 电机侧控制量
        const float denominator = Param[i].TorqueConstant * Param[i].ReductionRatio;
        if (denominator != 0.0f) {
            Out[i] = (torque / denominator) * (float)Param[i].Direction;
        } else {
            Out[i] = 0.0f;
        }

        PreVelocity[i] = Velocity[i];
    }
}

/**
 * @brief 清零全部状态量
 */
void ForceSolver::reset() {
    for (uint8_t j = 0U; j < DofNum; ++j) {
        TargetGeneralizedForce[j] = 0.0f;
    }

    for (uint8_t i = 0U; i < ActuatorNum; ++i) {
        FeedforwardTorque[i] = 0.0f;
        Position[i] = 0.0f;
        Velocity[i] = 0.0f;
        PreVelocity[i] = 0.0f;
        VelocityFilterOutput[i] = 0.0f;
        VelocityFilterReady[i] = false;
        Torque[i] = 0.0f;
        Out[i] = 0.0f;
    }
}

/**
 * @brief 生成四轮全向底盘的分配矩阵（最小范数解）
 *
 * @param wheelRadius          轮子半径 r, m
 * @param wheelCenterDistance  轮心到车体中心距离 L, m
 * @param matrix               输出缓冲，行优先 4 x 3，至少 12 个 float
 * @return true 生成成功
 */
bool ForceSolver::buildOmniDistribution(
    float wheelRadius,
    float wheelCenterDistance,
    float *matrix
) {
    if (matrix == nullptr || wheelCenterDistance == 0.0f || wheelRadius == 0.0f) {
        return (false);
    }

    // 各轮速度的 x / y 分量系数，轮子编号与 ChassisConsole 的四条式子一致
    static const float wheelCoefficientX[4] = {
        0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f
    };
    static const float wheelCoefficientY[4] = {
        -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f
    };

    // 平移项乘 1/2（B^T B 的平移对角元为 2），旋转项乘 1/(4L^2)
    const float translationScale = wheelRadius * 0.5f;
    const float rotationScale = -wheelRadius / (4.0f * wheelCenterDistance);

    for (uint8_t i = 0U; i < 4U; ++i) {
        matrix[i * 3U + 0U] = wheelCoefficientX[i] * translationScale;
        matrix[i * 3U + 1U] = wheelCoefficientY[i] * translationScale;
        matrix[i * 3U + 2U] = rotationScale;
    }

    return (true);
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/

/*----------------------------------------------------------------------------*/
/* PC 侧自检：只在定义 ALG_FORCE_HOST_TEST 时编译，固件构建不受影响            */
/*----------------------------------------------------------------------------*/

#ifdef ALG_FORCE_HOST_TEST

#include <stdio.h>

static int ForceSolverHostTestFailCount = 0;

/**
 * @brief 打印一条自检结果
 */
static void ForceSolverHostCheck(bool condition, const char *name, float got, float expect) {
    if (!condition) {
        ForceSolverHostTestFailCount++;
    }
    printf(
        "[%s] %-34s got=%+.6f  expect=%+.6f\n",
        condition ? "OK  " : "FAIL",
        name,
        (double)got,
        (double)expect
    );
}

/**
 * @brief 用例 1：四轮全向底盘，广义力 -> 轮力矩 -> 还原广义力
 *
 * 用最小范数分配矩阵解出的轮力矩，反推牵引力后再按机构学还原，
 * 应当得到最初的 (Fx, Fy, Mz)。这一条能直接抓出分配矩阵缩放错误。
 */
static void ForceSolverHostTestOmniChassis() {
    printf("---- 用例 1：四轮全向底盘，广义力 -> 轮力矩 -> 还原广义力 ----\n");

    const float wheelRadius = 0.05f;
    const float wheelCenterDistance = 0.25f;

    ForceActuatorParam_t param[4] = {};
    for (int i = 0; i < 4; ++i) {
        param[i].TorqueConstant = 1.0f;
        param[i].ReductionRatio = 1.0f;
        param[i].Direction = 1;
        param[i].TorqueMax = 100.0f;
    }

    ForceSolver solver;
    solver.init(param, 4U, 3U, 0.001f);

    float matrix[4 * 3] = {};
    const bool built = ForceSolver::buildOmniDistribution(wheelRadius, wheelCenterDistance, matrix);
    ForceSolverHostCheck(built, "buildOmniDistribution 返回 true", built ? 1.0f : 0.0f, 1.0f);
    solver.setDistribution(matrix);

    const float generalizedForce[3] = {10.0f, -4.0f, 1.5f};
    solver.setTarget(generalizedForce);
    solver.setEnable(true);
    solver.timCalculatePeriodElapsedCallback();

    static const float coefficientX[4] = {
        0.70710678f, 0.70710678f, -0.70710678f, -0.70710678f
    };
    static const float coefficientY[4] = {
        -0.70710678f, 0.70710678f, 0.70710678f, -0.70710678f
    };

    float forceX = 0.0f;
    float forceY = 0.0f;
    float momentZ = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i) {
        const float traction = solver.getTorque(i) / wheelRadius;
        forceX += coefficientX[i] * traction;
        forceY += coefficientY[i] * traction;
        momentZ += -wheelCenterDistance * traction;
    }

    ForceSolverHostCheck(
        fabsf(forceX - generalizedForce[0]) < 1e-3f, "还原 Fx", forceX, generalizedForce[0]
    );
    ForceSolverHostCheck(
        fabsf(forceY - generalizedForce[1]) < 1e-3f, "还原 Fy", forceY, generalizedForce[1]
    );
    ForceSolverHostCheck(
        fabsf(momentZ - generalizedForce[2]) < 1e-3f, "还原 Mz", momentZ, generalizedForce[2]
    );
}

/**
 * @brief 用例 2：单关节重力矩补偿与失能清零
 */
static void ForceSolverHostTestGravity() {
    printf("---- 用例 2：单关节重力矩补偿 ----\n");

    ForceActuatorParam_t param = {};
    param.TorqueConstant = 1.0f;
    param.ReductionRatio = 1.0f;
    param.Direction = 1;
    param.TorqueMax = 100.0f;
    param.GravityAmplitude = 2.0f;
    param.GravityOffset = 0.3f;

    ForceSolver solver;
    solver.init(&param, 1U, 1U, 0.001f);

    const float velocity[1] = {0.0f};
    const float positionOnAxis[1] = {0.3f};
    solver.setNow(positionOnAxis, velocity);
    solver.setEnable(true);
    solver.timCalculatePeriodElapsedCallback();

    ForceSolverHostCheck(
        fabsf(solver.getTorque(0) - 2.0f) < 1e-4f, "重力矩 A*cos(theta-phi)", solver.getTorque(0), 2.0f
    );

    // theta - phi = pi/2 时重力矩应为 0
    const float positionQuarter[1] = {param.GravityOffset + 1.57079633f};
    solver.setNow(positionQuarter, velocity);
    solver.timCalculatePeriodElapsedCallback();
    ForceSolverHostCheck(
        fabsf(solver.getTorque(0)) < 1e-4f, "重力矩在 pi/2 处为 0", solver.getTorque(0), 0.0f
    );

    // 失能后输出必须清零
    solver.setEnable(false);
    solver.timCalculatePeriodElapsedCallback();
    ForceSolverHostCheck(
        (solver.getTorque(0) == 0.0f) && (solver.getOut(0) == 0.0f),
        "失能后输出为 0",
        solver.getOut(0),
        0.0f
    );
}

/**
 * @brief 用例 3：力矩限幅与力矩 -> 电流换算
 */
static void ForceSolverHostTestConversion() {
    printf("---- 用例 3：力矩限幅与力矩 -> 电流换算 ----\n");

    ForceActuatorParam_t param = {};
    param.TorqueConstant = 0.7f;
    param.ReductionRatio = 19.0f;
    param.Direction = -1;
    param.TorqueMax = 1.0f;

    ForceSolver solver;
    solver.init(&param, 1U, 1U, 0.001f);

    const float generalizedForce[1] = {5.0f};
    solver.setTarget(generalizedForce);
    solver.setEnable(true);
    solver.timCalculatePeriodElapsedCallback();

    const float expectTorque = 1.0f;
    const float expectOut = -(1.0f / (0.7f * 19.0f));
    ForceSolverHostCheck(
        fabsf(solver.getTorque(0) - expectTorque) < 1e-5f,
        "力矩被 TorqueMax 钳住",
        solver.getTorque(0),
        expectTorque
    );
    ForceSolverHostCheck(
        fabsf(solver.getOut(0) - expectOut) < 1e-6f,
        "力矩 -> 电流换算与方向",
        solver.getOut(0),
        expectOut
    );
}

int main() {
    ForceSolverHostTestOmniChassis();
    ForceSolverHostTestGravity();
    ForceSolverHostTestConversion();

    printf("\n%s，失败 %d 项\n", (ForceSolverHostTestFailCount == 0) ? "全部通过" : "存在失败",
           ForceSolverHostTestFailCount);

    return (ForceSolverHostTestFailCount == 0) ? 0 : 1;
}

#endif