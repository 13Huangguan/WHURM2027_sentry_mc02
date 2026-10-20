/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.cpp/hpp
  * @brief      通过USB串口与上位机通信 - 任务层
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "usb_task.hpp"

#include "cmsis_os.h"
#include "usb_device.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
extern "C" void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成
    usb.Init();

    while (1) {
        usb.Update();

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*------------------------------ End of File ------------------------------*/
