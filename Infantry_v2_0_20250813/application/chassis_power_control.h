/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.h
  * @brief      ���̹��ʿ���
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
	
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "chassis_task.h"
#include "main.h"

#define TORQUE_to_Icmd 1.907e-5f	//�����н�����torqueת��Ϊ���Ƶ���Icmd��ϵ��

extern uint8_t robot_id ;

//�������ƽṹ��
typedef struct{
	float k1[4], k2[4], constant[4];	//����ģ�Ͳ���
	
	fp32 max_power_limit;					//Ĭ�����������
	fp32 available_max_power;				//���̵�ǰ���������
	fp32 current_power_buffer;				//��ǰ��������ʣ��ֵ
	fp32 pre_target_power[4];				//ÿ�����ֵ�Ԥ�⹦��
	fp32 pre_target_power_sum;				//��Ԥ�⹦��
	int16_t pre_rpm[4];						//Ԥ��ת��
	
}	CHASSIS_POWER_CONTROL_t;

/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_move);


#endif
