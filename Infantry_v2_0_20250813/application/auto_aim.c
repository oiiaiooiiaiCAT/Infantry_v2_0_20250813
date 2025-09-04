#include "main.h"
#include "bsp_usart.h"
#include "auto_aim.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "arm_math.h"

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "shoot.h"
#include "voltage_task.h"
#include "CRC8_CRC16.h"
#include "GRAVITY.h"
#include "tim.h"

static uint8_t send[PACKET_SIZE];

//�󲹳���yaw��
fp32 yaw;

// ���ڽ���ĸ����ṹ��ȷ���ֽ�����ȷ
typedef union {
    int16_t int16;
    uint8_t bytes[2];
} int16_bytes_t;

typedef struct {
    float yaw;
    float pitch;
    float distance;
    float shoot_delay;
} TargetPacket;

TargetPacket packet;
fp32 CompensationAngle = 0;
// Example auto_aim function
void auto_aim(uint8_t* data_send_from_pc)
{
	//���鿪
	if(!aim_flag){
		if (auto_aim_flag) {
      auto_aim_flag = 0;
			return; 
    }
			else{
			// 1. Check the start/end bytes
			if (data_send_from_pc[0] != START_BYTE ||
					data_send_from_pc[9] != END_BYTE) {
					return; // Format error
			}

			int16_t x_int = (data_send_from_pc[1] << 8) | data_send_from_pc[2];
			int16_t y_int = (data_send_from_pc[3] << 8) | data_send_from_pc[4];
			int16_t z_int = (data_send_from_pc[5] << 8) | data_send_from_pc[6];
			uint16_t shoot_delay_int = (data_send_from_pc[7] << 8) | data_send_from_pc[8];
			
			packet.distance    = (float)z_int * 1000.0f  / 32767.0f /100.0f; 
			
			if(packet.distance >= 0.1f){
					packet.distance = packet.distance / 1.57f; //��һ���������
			}
			//�ضϴ���
			if(packet.distance <1.5f ){
				//pitch�Ƿ���
				CompensationAngle = calculateCompensationAngle(gimbal_control.gimbal_pitch_motor.relative_angle,
																												packet.distance,
																												Initial_velocity,
																												params) ;
			
				
			}
			else if(packet.distance >1.5f && packet.distance <2.0f){
				CompensationAngle = calculateCompensationAngle(gimbal_control.gimbal_pitch_motor.relative_angle,
																												packet.distance,
																												Initial_velocity,
																												params) * 1.2f;
			}
			else if(packet.distance >2.0f && packet.distance <2.5f ){
				CompensationAngle = calculateCompensationAngle(gimbal_control.gimbal_pitch_motor.relative_angle,
																												packet.distance,
																												Initial_velocity,
																												params) * 1.5f;
			}
			packet.yaw         = (float)x_int * 100.0f / 32767.0f * AUTO_AIM_PPL * PI_RAD ; 
			packet.pitch       = (float)(y_int + CompensationAngle)* 100.0f / 32767.0f * AUTO_AIM_PPL * PI_RAD; 
			packet.yaw = - packet.yaw;
			packet.pitch = - packet.pitch;
			
			packet.shoot_delay = (float)shoot_delay_int; // if you intend raw to match directly
			
			if(packet.distance > 2.5f){
				if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE){
					packet.pitch = gimbal_control.gimbal_pitch_motor.min_relative_angle - gimbal_control.gimbal_pitch_motor.relative_angle;
				}
				else if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO){
					packet.pitch = gimbal_control.gimbal_pitch_motor.min_relative_angle - gimbal_control.gimbal_pitch_motor.absolute_angle;
				}
			}
			
			// 5. Then run your motor commands, CAN commands, etc.
			//    (Same logic you had, but with the corrected yaw/pitch/distance)
			 if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE){
					gimbal_relative_angle_limit(&gimbal_control.gimbal_pitch_motor, packet.pitch);
					gimbal_motor_relative_angle_control(&gimbal_control.gimbal_pitch_motor);
			}
			else if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO){
					gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, packet.pitch);
					gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_pitch_motor);
			}

			if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE){
					gimbal_relative_angle_limit(&gimbal_control.gimbal_pitch_motor, packet.yaw);
					gimbal_motor_relative_angle_control(&gimbal_control.gimbal_yaw_motor);
			}
			else if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO){
					gimbal_absolute_angle_limit(&gimbal_control.gimbal_yaw_motor, packet.yaw);
					gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_yaw_motor);
			}

			CAN_cmd_GIMB_6020(gimbal_control.gimbal_yaw_motor.given_current, 
												gimbal_control.gimbal_pitch_motor.given_current, 
												0, 0);
			CAN_cmd_GIMB_FP(shoot_can_set_current, shoot_can_set_current, 0, 0);
		}
	}
    // ...
    // Optionally send status back to PC, etc.
}

void send_to_minipc(void) {
    const gimbal_motor_t *pitch = get_pitch_motor_point();
    const gimbal_motor_t *yaw = get_yaw_motor_point();

    fp32_to_bytes pitch_data;
    fp32_to_bytes yaw_data;
    pitch_data.fp32 =  pitch->relative_angle;
    yaw_data.fp32 =  yaw->relative_angle;
	
    // ��װ���ݰ�
    send[0] = pitch_data.bytes[0];  // pitch �ĵ�1���ֽ�
    send[1] = pitch_data.bytes[1];  // pitch �ĵ�2���ֽ�
		send[2] = pitch_data.bytes[2];  // pitch �ĵ�1���ֽ�
    send[3] = pitch_data.bytes[3];  // pitch �ĵ�2���ֽ�
	
    send[4] = yaw_data.bytes[0];    // yaw �ĵ�1���ֽ�
    send[5] = yaw_data.bytes[1];    // yaw �ĵ�2���ֽ�
    send[6] = yaw_data.bytes[2];    // yaw �ĵ�1���ֽ�
    send[7] = yaw_data.bytes[3];    // yaw �ĵ�2���ֽ�
	
		send[8] = 'e'; //����-���-С��
	
		send[9] = 0x00; //��̨��־λ
		
		send[10] = 0x00; //�����ݱ�־λ
		//��3��id��3����3��67
		if(robot_id <50){
			send[11] = 0X00; //�Է���ɫ��ɫ
		}
		else{
			send[11] = 0X01; //�Է�����ɫ��������Ǻ�ɫ
		}
		send[12] = 0X00; //��������x�Ჹ��
		send[13] = 0X00;
		
		send[14] = 0X00; //��������y�Ჹ��
		send[15] = 0X10;
		
    // �������ݰ�
    // usart1_tx_dma_enable(send, sizeof(send));
    usart6_tx_dma_enable(send, sizeof(send));
}
