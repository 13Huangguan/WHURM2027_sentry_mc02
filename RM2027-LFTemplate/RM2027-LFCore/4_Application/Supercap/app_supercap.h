#pragma once
#include "core_linkage.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SUPERCAP_NOT_CONFIGURED = 0,
    SUPERCAP_WAIT_REFEREE,
    SUPERCAP_WAIT_FEEDBACK,
    SUPERCAP_READY,
    SUPERCAP_REFEREE_OFFLINE,
    SUPERCAP_INIT_FAILED
} SupercapAppStatus;

/** @brief 仅初始化设备，由 freertos.c 在创建线程前调用；不在此创建线程。 */
LF_EXTERN_C bool supercapInit(void);
/** @brief FreeRTOS 任务入口，线程定义和创建均位于 Core/Src/freertos.c。 */
LF_EXTERN_C void superpower_task(void const* argument);
/** @brief 默认请求使能，与提供的任务一致；实际发送要求已有有效且新鲜的裁判数据。 */
LF_EXTERN_C void supercapSetEnabled(bool enabled);
LF_EXTERN_C SupercapAppStatus supercapGetStatus(void);
/** @brief 在线、新鲜裁判数据就绪时提供超电反馈的底盘预算，失败置零。 */
LF_EXTERN_C bool supercapGetChassisPowerLimit(float* watts);

#ifdef __cplusplus
#include "dvc_supercap.hpp"
/** @brief 直接读取设备反馈，应用层不另存反馈副本。 */
SupercapFeedback supercapGetFeedback(void);
#endif
