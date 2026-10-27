#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Chassis_Init(void);
void Chassis_Control(void);

#ifdef __cplusplus
}
#endif

#endif
