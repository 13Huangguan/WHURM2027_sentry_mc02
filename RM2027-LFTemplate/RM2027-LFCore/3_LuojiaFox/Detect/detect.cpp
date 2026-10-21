#include "detect.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"
#include "dvc_bmi088.hpp"  //注册BMI检测

static error_t error_list[DETECT_COUNT + 1];       //检测状态列表，最后一项用作所有错误的汇总状态
static constexpr uint32_t kDetectPeriodMs = 10U;   //检测周期为10ms

static void detectInit(uint32_t now);              //检测初始化函数
static void detectPoll(uint32_t now);              //检测任务内部的周期巡检函数

static void detectInit(uint32_t now)
{   //初始化参数
    for (uint8_t i = 0; i < DETECT_COUNT; ++i) {
        error_list[i] = {};
        error_list[i].enable = 1;
        error_list[i].error_exist = true;
        error_list[i].is_lost = true;
        error_list[i].data_is_error = true;
        error_list[i].new_time = now;
        error_list[i].last_time = now;
        error_list[i].lost_time = now;
        error_list[i].work_time = now;
    }
    //------------------RC遥控器设置---------------------------------------------------------------------------------
    /* 单位全部是毫秒 */
    error_list[DETECT_RC].offline_time_ms = 100;
    error_list[DETECT_RC].online_time_ms = 100;
    //-------------------测试GM6020设置(之后直接删除)-------------------------------------------------------------------
    error_list[DETECT_PITCH_MOTOR].offline_time_ms = 10;
    error_list[DETECT_PITCH_MOTOR].online_time_ms = 20;
    //-------------------底盘轮设置-----------------------------------------------------------------------------------
    for (uint8_t i = DETECT_WHEEL1; i <= DETECT_WHEEL4; ++i) {
        error_list[i].offline_time_ms = 10;
        error_list[i].online_time_ms = 20;
    }
    //-----------------BMI设置----------------------------------------------------------------------------------------
    error_list[DETECT_BMI_GYRO].offline_time_ms = 10;
    error_list[DETECT_BMI_GYRO].online_time_ms = 20;
    error_list[DETECT_BMI_GYRO].data_is_error_fun =
    bmi088GyroDataError;

    error_list[DETECT_BMI_ACCEL].offline_time_ms = 10;
    error_list[DETECT_BMI_ACCEL].online_time_ms = 20;
    error_list[DETECT_BMI_ACCEL].data_is_error_fun =
    bmi088AccelDataError;

    /* USB 上位机还没接通，先禁用 */
    error_list[DETECT_NUC].enable = 0;
}


//表明已接收到toe的数据
extern "C" void detect_hook(uint8_t toe)
{
    if (toe >= DETECT_COUNT) {
        return;
    }

    error_t &item = error_list[toe];

    item.last_time = item.new_time;
    item.new_time = HAL_GetTick();

    if (item.is_lost) {
        item.is_lost = false;
        item.work_time = item.new_time;
    }
}

//检测任务内部的周期巡检函数
static void detectPoll(uint32_t now)
{
    for (uint8_t i = 0; i < DETECT_COUNT; ++i) {
        error_t &item = error_list[i];

        if (!item.enable) {
            continue;
        }
        //让掉线保护只在“第一次发现掉线”时执行一次，避免每 10 ms 重复执行恢复函数
        if ((now - item.new_time) > item.offline_time_ms) {
            if (!item.is_lost) {
                item.is_lost = true;
                item.lost_time = now;

                if (item.solve_lost_fun != nullptr) {
                    item.solve_lost_fun();
                }
            }

            item.error_exist = true;
            continue;
        }

        item.is_lost = false;

        if ((now - item.work_time) < item.online_time_ms) {
            /* 刚上线，仍保持错误状态，但不判定为掉线 */
            item.error_exist = true;
            continue;
        }

        if (item.data_is_error_fun != nullptr) {
            item.data_is_error = item.data_is_error_fun();

            if (item.data_is_error && item.solve_data_error_fun != nullptr) {
                item.solve_data_error_fun();
            }
        } else {
            item.data_is_error = false;
        }

        item.error_exist = item.data_is_error;

        if (item.new_time > item.last_time) {
            item.frequency =
                1000.0f / static_cast<float>(
                    item.new_time - item.last_time
                );
        }
    }
}

extern "C" void Detect_Task(void const *argument)
{
    (void)argument;

    detectInit(HAL_GetTick());
    osDelay(50);

    for (;;) {
        detectPoll(HAL_GetTick());
        osDelay(kDetectPeriodMs);
    }
}

extern "C" bool toe_is_error(uint8_t toe)
{
    if (toe >= DETECT_COUNT) {
        return true;
    }

    return error_list[toe].error_exist;
}

extern "C" const error_t *get_error_list_point(void)
{
    return error_list;
}