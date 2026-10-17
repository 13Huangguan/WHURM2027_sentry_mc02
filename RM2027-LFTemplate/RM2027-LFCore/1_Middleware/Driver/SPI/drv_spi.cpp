/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_spi.c
  * @brief          : bsp spi functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "drv_spi.h"

#include "spi.h"

/**
  * @brief returns the spi receive data after transmiting the specified data 
  * @param Tx_Data: the specified data
  * @retval the spi receive data
  */
uint8_t bmi088ReadWriteByte(uint8_t tx_data) {
    uint8_t rx_data = 0;

    HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1, 100);

    return rx_data;
}