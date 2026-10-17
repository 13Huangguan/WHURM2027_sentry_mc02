#include "dvc_bsp.h"




void bspInit(void)
{
  // BMI088_Init() moved to INS_Task to avoid HardFault during pre-scheduler initialization
  // The issue was Delay_ms() using HAL_GetTick() before FreeRTOS scheduler starts
}