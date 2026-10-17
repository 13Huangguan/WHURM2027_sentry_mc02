/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LPF.c
  * @brief          : lowpass filter
  * @author         : GrassFan Wang
  * @date           : 2025/12/28
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "alg_lpf.hpp"

#include "string.h"

/**
  * @brief 初始化一阶低通滤波器.
  * @param LPF: 一阶低通滤波器结构体.
  * @param Alpha: 滤波器系数.
  * @param Frame_Period: 采样周期.
  * @retval 无.
  */
void lowPassFilter1pInit(LowPassFilter1p* lpf, float alpha) {
    lpf->initialized = false;
    lpf->alpha = alpha;
    lpf->input = 0;
    lpf->output = 0;
}

/**
  * @brief 更新一阶低通滤波器输出.
  * @param Input: 当前输入.
  * @retval 滤波器输出.
  */
float lowPassFilter1pUpdate(LowPassFilter1p* lpf, float input) {
    lpf->input = input;

    if (lpf->initialized == false) {
        lpf->output = lpf->input;
        lpf->initialized = true;
    }

    lpf->output = lpf->alpha * lpf->output + (1.f - lpf->alpha) * lpf->input;

    return lpf->output;
}

/**
  * @brief 初始化二阶低通滤波器.
  * @param Alpha[3]: 滤波器系数[3].
  * @retval 无.
  */
void lowPassFilter2pInit(LowPassFilter2p* lpf, const float alpha[3]) {
    lpf->initialized = false;
    memcpy(lpf->alpha, alpha, sizeof(lpf->alpha));
    lpf->input = 0;
    memset(lpf->output, 0, sizeof(lpf->output));
}

/**
  * @brief 更新二阶低通滤波器输出.
  * @param Input: 当前输入.
  * @retval 滤波器输出.
  */
float lowPassFilter2pUpdate(LowPassFilter2p* lpf, float input) {
    lpf->input = input;

    if (lpf->initialized == false) {
        lpf->output[0] = lpf->input;
        lpf->output[1] = lpf->input;
        lpf->output[2] = lpf->input;
        lpf->initialized = true;
    }

    lpf->output[0] = lpf->output[1];
    lpf->output[1] = lpf->output[2];
    lpf->output[2] = lpf->alpha[0] * lpf->output[1] + lpf->alpha[1] * lpf->output[0]
        + lpf->alpha[2] * lpf->input;

    return lpf->output[2];
}