/**
 * @file alg_basic.h
 * @author Bayekq
 * @brief 一些极其简易的数学（使用小驼峰命名）
 * @version 1.0
 * @date 2026-09-25 1.0 初始版本，参考中科大编写
 *
 * @copyright Copyright (c) 2026-2027
 *
 */


#ifndef ALG_BASIC_H
#define ALG_BASIC_H

/* Includes ------------------------------------------------------------------*/

#include <float.h>

#include "arm_math.h"

/* Exported macros -----------------------------------------------------------*/

extern const float BasicMathRpmToRadps;
extern const float BasicMathDegToRad;
extern const float BasicMathCelsiusToKelvin;

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

namespace basic_math {

void booleanLogicalNot(bool* value);

void endianReverse16(void* address);

uint16_t endianReverse16(void* source, void* destination);

void endianReverse32(void* address);

uint32_t endianReverse32(void* source, void* destination);

uint8_t sum8(const uint8_t* address, uint32_t length);

uint16_t sum16(const uint16_t* address, uint32_t length);

uint32_t sum32(const uint32_t* address, uint32_t length);

float sinc(float x);

int32_t floatToInt(float x, float float1, float float2,
                   int32_t int1, int32_t int2);

float intToFloat(int32_t x, int32_t int1, int32_t int2,
                 float float1, float float2);

bool isInvalidFloat(float x);

float modulusNormalization(float x, float modulus);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 * @return 输出值
 */
template <typename Type>
Type constrain(Type x, Type min, Type max) {
  if (x < min) {
    x = min;
  } else if (x > max) {
    x = max;
  }
  return (x);
}

/**
 * @brief 限幅函数（指针版本）
 *
 * @tparam Type 类型
 * @param x 传入数据指针
 * @param min 最小值
 * @param max 最大值
 * @return 输出值
 */
template <typename Type>
Type constrain(Type* x, Type min, Type max) {
  if (*x < min) {
    *x = min;
  } else if (*x > max) {
    *x = max;
  }
  return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type abs(Type x) {
  return ((x > 0) ? x : -x);
}

/**
 * @brief 求符号
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return int8_t 返回 1 / 0 / -1
 */
template <typename Type>
int8_t sign(Type x) {
  if (x > 0) {
    return (1);
  } else if (x < 0) {
    return (-1);
  }
  return (0);
}

/**
 * @brief 正弦，基于 CMSIS-DSP 的查表实现
 *
 * @param x 弧度值
 * @return float sin(x)
 */
float sin(float x);

/**
 * @brief 余弦，基于 CMSIS-DSP 的查表实现
 *
 * @param x 弧度值
 * @return float cos(x)
 */
float cos(float x);
}  // namespace basic_math

#endif

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/