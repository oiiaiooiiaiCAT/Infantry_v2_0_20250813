#include "bsp_usart.h"
#include "main.h"
#include "referee_usart_task.h"
#include "usb_task.h"
#include "bsp_buzzer.h"
#include "auto_aim.h"
#include "string.h"
#include "chassis_power_control.h"
#include "auto_aim.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

//#define DMA_FLAG_TCIF5 ((uint32_t)0x20000800)
static uint8_t gimbal_data[USART_BUF_LENGHT / 2]={0};
uint8_t usart1_buf[2][ USART_BUF_LENGHT ];//����˫������
uint8_t data_send_from_pc[ USART_BUF_LENGHT/2] = {0};
uint8_t data_send_from_pc6[ USART_BUF_LENGHT ] = {0};
uint8_t data_send_from_chassis[ USART_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//�������·�������

void usart1_init(void)
{
	//ʹ��DMA���ڽ��պͷ���
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	//�����ж�
	
	//ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	
	while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF7);

	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
	//�ڴ滺����1
	hdma_usart1_rx.Instance->M0AR = (uint32_t)(usart1_buf[0]);
	//�ڴ滺����2
	hdma_usart1_rx.Instance->M1AR = (uint32_t)(usart1_buf[1]);
	//���ݳ���3
	__HAL_DMA_SET_COUNTER(&hdma_usart1_rx, USART_BUF_LENGHT);
	//ʹ��˫������
	SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
	
	//ʹ��DMA
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
	
	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
}

//���̡���̨�����ݽ���
void uart1_data_receive(void)
{
	if(USART1->SR & UART_FLAG_IDLE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);

		static uint16_t this_time_rx_len = 0;
		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = USART_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

			hdma_usart1_rx.Instance->NDTR = USART_BUF_LENGHT;

			//�趨������1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
			
			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			if(this_time_rx_len ==  15u){
				#ifdef chassis_board
					memcpy(gimbal_data,usart1_buf[0] , 15);
					gimbal_to_chassis(gimbal_data);
				#endif
				#ifdef gimbal_board
				//���̷��͵���Ϣ
					memcpy(gimbal_data,usart1_buf[0], 15);
					chassis_to_gimbal(gimbal_data);
				#endif
			}
		}
		else{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = USART_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

			hdma_usart1_rx.Instance->NDTR = USART_BUF_LENGHT;

			//�趨������0
			DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
			
			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			if(this_time_rx_len ==  14u){
				//������
				#ifdef chassis_board
					memcpy(gimbal_data,usart1_buf[1] , 10);
					//����������̨���͵���Ϣ
					gimbal_to_chassis(gimbal_data);
				#endif
				#ifdef gimbal_board
				//���̷��͵���Ϣ
					memcpy(gimbal_data,usart1_buf[0], 14);
					chassis_to_gimbal(gimbal_data);
				#endif
			}
		}
	}
}

void usart1_receive(void){
	// ��ʼDMA����
	HAL_UART_Receive_DMA(&huart1, usart1_buf[0], USART_BUF_LENGHT/2);
}

void usart1_tx_dma_init(void)
{
	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
	hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
	hdma_usart1_tx.Instance->NDTR = 0;
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
	//disable DMA
	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

	hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

	__HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//ʹ��DMA���ڽ��պͷ���
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

	//ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	
	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	//�ڴ滺����1
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
	//�ڴ滺����2
	hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	//���ݳ���
	__HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

	//ʹ��˫������
	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

	//ʹ��DMA
	__HAL_DMA_ENABLE(&hdma_usart6_rx);

	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

//����6�Ļص�����
void usart6_rx_complete_callback(uint8_t *buf, uint16_t len) {
	// ����֤���ݰ���ʽ�Ƿ���ȷ
  // �������յ������ݵ�data_send_from_pc
  if (buf[0] == 's' || buf[9] == 'e') {
		memcpy(data_send_from_pc6, buf, len);
		auto_aim(data_send_from_pc6);
  }
  else{
    usart6_tx_dma_enable(restart_array,len);
  }
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
	//ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

	__HAL_DMA_ENABLE(&hdma_usart6_tx);
}


fp32 chassis_INS_yaw, chassis_INS_pitch, chassis_spin_speed;	//��Щ������������ĺ���

//��̨������̷���������
void chassis_to_gimbal(uint8_t *gimbal_data){
  fp32_to_bytes yaw;
	yaw.bytes[0] = gimbal_data[1]; 
	yaw.bytes[1] = gimbal_data[2]; 
	yaw.bytes[2] = gimbal_data[3]; 
	yaw.bytes[3] = gimbal_data[4]; 
  chassis_INS_yaw = yaw.fp32;

  fp32_to_bytes pitch;
	pitch.bytes[0] = gimbal_data[5]; 
	pitch.bytes[1] = gimbal_data[6]; 
	pitch.bytes[2] = gimbal_data[7]; 
	pitch.bytes[3] = gimbal_data[8]; 
	chassis_INS_pitch = pitch.fp32;
	
  fp32_to_bytes roll;
	roll.bytes[0] = gimbal_data[9]; 
	roll.bytes[1] = gimbal_data[10]; 
	roll.bytes[2] = gimbal_data[11]; 
	roll.bytes[3] = gimbal_data[12];
  chassis_spin_speed = roll.fp32;
	robot_id = gimbal_data[13];
}

fp32 yaw_motor_relative_angle, GIMBAL_INS_yaw;	//��Щ������������ĺ���

//���̴�����̨����������
void gimbal_to_chassis(uint8_t *gimbal_data) {
	fp32_to_bytes INS;
	INS.bytes[0] = gimbal_data[1]; 
	INS.bytes[1] = gimbal_data[2]; 
	INS.bytes[2] = gimbal_data[3]; 
	INS.bytes[3] = gimbal_data[4]; 
	GIMBAL_INS_yaw = INS.fp32;

	fp32_to_bytes Motor;
	Motor.bytes[0] = gimbal_data[5]; 
	Motor.bytes[1] = gimbal_data[6]; 
	Motor.bytes[2] = gimbal_data[7]; 
	Motor.bytes[3] = gimbal_data[8]; 
	yaw_motor_relative_angle = Motor.fp32;
}
