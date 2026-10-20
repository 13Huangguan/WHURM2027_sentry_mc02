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
#ifndef USB_TASK_HPP
#define USB_TASK_HPP

#include "usb.hpp"

#define USB_TASK_CONTROL_TIME 1  // ms

/* 线程在 Core/Src/freertos.c（C 文件）里注册，必须保持 C 链接 */
extern "C" void usb_task(void const * argument);

#endif
/*------------------------------ End of File ------------------------------*/
