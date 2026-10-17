/**
 * @file alg_basic.cpp
 * @author Bayekq
 * @brief 一些极其简易的数学
 * @version 1.0
 * @date 2026/9/25 1.0 初始版本，参考中科大编写
 *
 * @copyright Copyright (c) 2026-2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.hpp"

#include <cmath>
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// rpm换算到rad/s
const float BASIC_MATH_RPM_TO_RADPS = 2.0f * PI / 60.0f;
// deg换算到rad
const float BASIC_MATH_DEG_TO_RAD = PI / 180.0f;
const float BASIC_MATH_RAD_TO_DEG = 180.0f / PI;
// 摄氏度换算到开氏度
const float BASIC_MATH_CELSIUS_TO_KELVIN = 273.15f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param value 布尔值地址
 */
void basic_math::booleanLogicalNot(bool* value) {
    if (!*value) {
        *value = true;
    } else if (*value) {
        *value = false;
    }
}

/**
 * @brief 16位大小端转换（原地版本）
 *
 * @param address 地址
 */
void basic_math::endianReverse16(void* address) {
    uint8_t* tmp_address8;
    uint16_t* tmp_address16;
    tmp_address8 = (uint8_t*)address;
    tmp_address16 = (uint16_t*)address;
    *tmp_address16 = tmp_address8[0] << 8 | tmp_address8[1];
}

/**
 * @brief 16位大小端转换（源到目标版本）
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t basic_math::endianReverse16(void* source, void* destination) {
    uint8_t* tmp_address8;
    uint16_t tmp_value16;
    tmp_address8 = (uint8_t*)source;
    tmp_value16 = tmp_address8[0] << 8 | tmp_address8[1];

    if (destination != nullptr) {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t*)source;
        tmp_destination = (uint8_t*)destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return tmp_value16;
}

/**
 * @brief 32位大小端转换（原地版本）
 *
 * @param address 地址
 */
void basic_math::endianReverse32(void* address) {
    uint8_t* tmp_address8;
    uint32_t* tmp_address32;
    tmp_address8 = (uint8_t*)address;
    tmp_address32 = (uint32_t*)address;
    *tmp_address32 =
        tmp_address8[0] << 24 | tmp_address8[1] << 16 | tmp_address8[2] << 8 | tmp_address8[3];
}

/**
 * @brief 32位大小端转换（源到目标版本）
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t basic_math::endianReverse32(void* source, void* destination) {
    uint8_t* tmp_address8;
    uint32_t tmp_value32;
    tmp_address8 = (uint8_t*)source;
    tmp_value32 =
        tmp_address8[0] << 24 | tmp_address8[1] << 16 | tmp_address8[2] << 8 | tmp_address8[3];

    if (destination != nullptr) {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t*)source;
        tmp_destination = (uint8_t*)destination;
        tmp_destination[0] = tmp_source[3];
        tmp_destination[1] = tmp_source[2];
        tmp_destination[2] = tmp_source[1];
        tmp_destination[3] = tmp_source[0];
    }

    return tmp_value32;
}

/**
 * @brief 8位数据求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t basic_math::sum8(const uint8_t* address, uint32_t length) {
    uint8_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum += address[i];
    }
    return sum;
}

/**
 * @brief 16位数据求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t basic_math::sum16(const uint16_t* address, uint32_t length) {
    uint16_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum += address[i];
    }
    return sum;
}

/**
 * @brief 32位数据求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t basic_math::sum32(const uint32_t* address, uint32_t length) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum += address[i];
    }
    return sum;
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float basic_math::sinc(float x) {
    // 分母为0则按极限求法
    if (basic_math::abs(x) <= 2.0f * FLT_EPSILON) {
        return 1.0f;
    }

    return arm_sin_f32(x) / x;
}

/**
 * @brief 正弦，基于 CMSIS-DSP 的查表实现
 *
 * @param x 弧度值
 * @return float sin(x)
 */
float basic_math::sin(float x) {
    return arm_sin_f32(x);
}

/**
 * @brief 余弦，基于 CMSIS-DSP 的查表实现
 *
 * @param x 弧度值
 * @return float cos(x)
 */
float basic_math::cos(float x) {
    return arm_cos_f32(x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param float1 浮点数1
 * @param float2 浮点数2
 * @param int1 整型1
 * @param int2 整型2
 * @return int32_t 整型
 */
int32_t basic_math::floatToInt(float x, float float1, float float2, int32_t int1, int32_t int2) {
    float tmp = (x - float1) / (float2 - float1);
    auto out = (int32_t)(tmp * (float)(int2 - int1) + (float)(int1));
    return out;
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param int1 整型1
 * @param int2 整型2
 * @param float1 浮点数1
 * @param float2 浮点数2
 * @return float 浮点数
 */
float basic_math::intToFloat(int32_t x, int32_t int1, int32_t int2, float float1, float float2) {
    float tmp = (float)(x - int1) / (float)(int2 - int1);
    float out = tmp * (float2 - float1) + float1;
    return out;
}

/**
 * @brief 判断浮点数是否为无效浮点数
 *
 * @param x 浮点数
 * @return 是否为NaN或非正规数
 */
bool basic_math::isInvalidFloat(float x) {
    uint32_t bits;
    std::memcpy(&bits, &x, sizeof(bits));
    uint32_t exp = (bits >> 23) & 0xff;
    uint32_t frac = bits & 0x7fffff;
    if (exp == 0x00) {
        if (frac == 0x00) {
            // 正负零
            return false;
        }
        // 次正规数和非正规数
        return true;
    }
    if (exp == 0xFF) {
        // 无穷大和NaN
        return true;
    }
    // 正规数
    return false;
}

/**
 * @brief 求取模归化
 *
 * @param x 传入数据
 * @param modulus 模数
 * @return 返回的归化数, 介于 ±modulus / 2 之间
 */
float basic_math::modulusNormalization(float x, float modulus) {
    float tmp;

    tmp = fmodf(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f) {
        tmp += modulus;
    }

    return tmp - modulus / 2.0f;
}

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
float basic_math::inverseSqrt(float value) {
    return std::isfinite(value) && value > 0.0f ? 1.0f / std::sqrt(value) : 0.0f;
}
