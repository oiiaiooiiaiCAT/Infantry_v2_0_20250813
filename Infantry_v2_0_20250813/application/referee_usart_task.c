/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c
  * @brief      RM裁判系统数据处理
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/
	
#include "referee_usart_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"


extern UART_HandleTypeDef huart6;

unpack_data_t referee_unpack_obj;		//裁判系统数据解包结构体变量
fifo_s_t referee_fifo;							//裁判系统先进先出结构体变量
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];					//串口接收双缓冲区数组
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];	//从裁判系统获取的数据数组

fp32 energy[2];

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

  while(fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
			
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
//void referee_usart_task(void const * argument)
void referee_usart_task(void)
{
	init_referee_data();
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

	while(1)
	{
		referee_unpack_fifo_data();
		get_chassis_power_and_buffer((energy), (energy + 1));
		HAL_Delay(10);
//		osDelay(10);
	}
}

static uint16_t this_time_rx_len = 0;

void USART6_IRQHandler(void)
{
  #ifdef chassis_board
    if(USART6->SR & UART_FLAG_IDLE)
    {
			__HAL_UART_CLEAR_PEFLAG(&huart6);
			
			if((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				__HAL_DMA_DISABLE(huart6.hdmarx);
				this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
				__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
				huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
				__HAL_DMA_ENABLE(huart6.hdmarx);
				fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
//				detect_hook(REFEREE_TOE);
			}
			else
			{
				__HAL_DMA_DISABLE(huart6.hdmarx);
				this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
				__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
				huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
				__HAL_DMA_ENABLE(huart6.hdmarx);
				fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
//				detect_hook(REFEREE_TOE);
			}
    }
  #endif // DEBUG
  
  #ifdef gimbal_board
		if(USART6->SR & UART_FLAG_IDLE)
		{
			__HAL_UART_CLEAR_PEFLAG(&huart6);
			static uint16_t this_time_rx_len = 0;

			if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				__HAL_DMA_DISABLE(huart6.hdmarx);
				this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
				__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT  );
				huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
				__HAL_DMA_ENABLE(huart6.hdmarx);
				if(this_time_rx_len == 10u){
					usart6_rx_complete_callback(usart6_buf[0], this_time_rx_len);
				}
			}
			else
			{
				__HAL_DMA_DISABLE(huart6.hdmarx);
				this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
				__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT  );
				huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
				__HAL_DMA_ENABLE(huart6.hdmarx);
				if(this_time_rx_len == 10u ){
					usart6_rx_complete_callback(usart6_buf[1], this_time_rx_len);
				}
			}
		}
  #endif // DEBUG
}
