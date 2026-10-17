/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       app_referee.cpp/hpp
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "app_referee.hpp"

#include "cmsis_os.h"
#include "main.h"

#include "drv_uart.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"

#include "usb.hpp"
#include "LF.hpp"

extern UART_HandleTypeDef huart1;

typedef struct __packed
{
  uint16_t data_cmd_id, data_sub_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint32_t sentry_cmd;
} sentry_data_t;

typedef struct __packed
{
    SendFrameHeader_t frame_header;
    sentry_data_t data;
    uint16_t crc;
} SendDataSentryCmd_s;

static SendDataSentryCmd_s send_data_sentry_cmd;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern "C" void referee_task(void const * argument)
{
    referee.Init();
    fifo_s_init(&referee.fifo, referee.fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart1_init(referee.uart_buf[0], referee.uart_buf[1], USART_RX_BUF_LENGHT);

    send_data_sentry_cmd.frame_header.sof = 0xA5;
    send_data_sentry_cmd.frame_header.len = 10;
    send_data_sentry_cmd.frame_header.id = 0x00;
    send_data_sentry_cmd.data.data_cmd_id = 0x0301;
    send_data_sentry_cmd.data.data_sub_id = 0x0120;
    send_data_sentry_cmd.data.sender_id = (uint16_t)referee.GetRobotId();
    send_data_sentry_cmd.data.receiver_id = 0x8080;

    while(1)
    {

        send_data_sentry_cmd.frame_header.id = send_data_sentry_cmd.frame_header.id + 1;  // 每次发送数据帧时，帧ID递增
        send_data_sentry_cmd.data.sender_id = (uint16_t)referee.GetRobotId();
        //send_data_sentry_cmd.data.sentry_cmd = usb.GetSentryCmd();
        append_CRC8_check_sum((unsigned char *)&send_data_sentry_cmd.frame_header, sizeof(SendFrameHeader_t));
        append_CRC16_check_sum((uint8_t *)&send_data_sentry_cmd, sizeof(SendDataSentryCmd_s));

        HAL_UART_Transmit(&huart1, (uint8_t *)&send_data_sentry_cmd, sizeof(SendDataSentryCmd_s),100);
        while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);


        referee.UnpackFifoData();

        LuojiaFox::system_state.UpdateReferee({
          (uint16_t)referee.GetRobotId(),
          referee.GetCurrentHP(),
          !referee.IsOffline()
        });

        osDelay(10);
    }
}

/**
  * @brief          USART1 idle-line interrupt, feeds the DMA buffer into the fifo.
  * @note           中断向量表在 startup_stm32h723xx.s 里按 USART1_IRQHandler 这个名字
  *                 找入口，必须保持 C 链接，否则会落到向量表末尾那个 .weak Default_Handler。
  */
extern "C" void USART1_IRQHandler(void)
{
    //static volatile uint8_t res;
    if(USART1->ISR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((((DMA_Stream_TypeDef* )huart1.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            ((DMA_Stream_TypeDef* )huart1.hdmarx->Instance)->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            referee.FeedBytes((char*)referee.uart_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            ((DMA_Stream_TypeDef* )huart1.hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            referee.FeedBytes((char*)referee.uart_buf[1], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
    }
}

/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
	 //HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
		uint8_t *pData=&ch;
		HAL_StatusTypeDef state=HAL_UART_Transmit(&huart1, pData, sizeof(ch),10);//无符号char也是1byte=8bit
		if(state==HAL_OK){
				//提示发送成功
		}
}
