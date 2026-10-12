/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
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
#include "app_referee.h"
#include "main.h"
#include "cmsis_os.h"

#include "drv_uart.h"
//#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

//#include "usb_task.h"



/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

void sentryCmdSend(uint32_t);
 
extern UART_HandleTypeDef huart1;

uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

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

SendDataSentryCmd_s send_data_sentry_cmd;

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
void referee_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

    send_data_sentry_cmd.frame_header.sof = 0xA5;
    send_data_sentry_cmd.frame_header.len = 10;
    send_data_sentry_cmd.frame_header.id = 0x00;
    send_data_sentry_cmd.data.data_cmd_id = 0x0301;
    send_data_sentry_cmd.data.data_sub_id = 0x0120;
    send_data_sentry_cmd.data.sender_id = (uint16_t)get_robot_id();
    send_data_sentry_cmd.data.receiver_id = 0x8080;

    while(1)
    {
      
        send_data_sentry_cmd.frame_header.id = send_data_sentry_cmd.frame_header.id + 1;  // 每次发送数据帧时，帧ID递增
        send_data_sentry_cmd.data.sender_id = (uint16_t)get_robot_id();
        //send_data_sentry_cmd.data.sentry_cmd = SENTRY_CMD_DATA;
        append_CRC8_check_sum((unsigned char *)&send_data_sentry_cmd.frame_header, sizeof(SendFrameHeader_t));
        append_CRC16_check_sum((uint8_t *)&send_data_sentry_cmd, sizeof(SendDataSentryCmd_s));
        
        HAL_UART_Transmit(&huart1, (uint8_t *)&send_data_sentry_cmd, sizeof(SendDataSentryCmd_s),100);
        while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);


        referee_unpack_fifo_data();
        osDelay(10);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case RSTEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = RSTEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case RSTEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = RSTEP_LENGTH_HIGH;
      }break;
      
      case RSTEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = RSTEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = RSTEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case RSTEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = RSTEP_HEADER_CRC8;
      }break;

      case RSTEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = RSTEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = RSTEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case RSTEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = RSTEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = RSTEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART1_IRQHandler(void)
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
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            ((DMA_Stream_TypeDef* )huart1.hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[1], this_time_rx_len);
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


