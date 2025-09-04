/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c
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
	
#include "arm_math.h"
#include "capacitor_task.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "referee.h"
#include "voltage_task.h"
#include <stdbool.h>

extern RC_ctrl_t rc_ctrl;
extern CAP_INFO_t cap_info;

CHASSIS_POWER_CONTROL_t *chassis_pwr_ctrl;

fp32 model_coefficients[12] = {0};
bool Init_Done = false;
uint8_t robot_id = 0;

/**
 * @brief		���ʿ��ƺ�����ʼ�������ڸ�����ģ��ϵ����ֵ
 * @param		None
 * @retval	None
 */
static void Chassis_PowerCtrl_Init(float *model_coefficients)
{
	for(uint8_t i = 0; i < 4; i ++)
	{
		chassis_pwr_ctrl->k1[i] = model_coefficients[i * 3];
		chassis_pwr_ctrl->k2[i] = model_coefficients[i * 3 + 1];
		chassis_pwr_ctrl->constant[i] = model_coefficients[i * 3 + 2];
	}
	
	Init_Done = true;
}

/**
	* @brief					���ʿ��ƣ���ʵ��Ϊÿ���������������������ù���
  * @param[in]      chassis_pwr_ctrl: ��������
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_move)
{
	//ֻ���ʼ��һ�飬ȷ������ģ�͵�ϵ��
	if(!Init_Done) Chassis_PowerCtrl_Init(model_coefficients);
		
	//�Ӳ���ϵͳ��ȡ��������Ʋ���ȡ��ǰ��������ʣ��ֵ
	get_chassis_power_and_buffer(&chassis_pwr_ctrl->max_power_limit, &chassis_pwr_ctrl->current_power_buffer);
	
	//�жϻ�������ʣ��ֵ�Ƿ����10
	if(chassis_pwr_ctrl->current_power_buffer <= 10.0f) chassis_pwr_ctrl->current_power_buffer = 0.0f;
	
	//�Ȼ��ڻ�������������̵�ǰ��������ʣ�Ϊÿ��������Ԥ��4W�Ĺ���
	chassis_pwr_ctrl->available_max_power = chassis_pwr_ctrl->max_power_limit + chassis_pwr_ctrl->current_power_buffer - 16;
	
	//ͨ��ң��������Q/E�л���������״̬
	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
	{
		cap_info.cap_status = 0;	//�رճ�������
	}
	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		cap_info.cap_status = 1;	//������������
	}
	
	//�ٻ��ڳ������ݼ�����̵�ǰ���������
	//������ݵ�������5%
	if (cap_info.energy_percent > 5)
	{
		if (cap_info.cap_status == 0)
		{
			//����ģʽ���Ը��������ù���5W
			chassis_pwr_ctrl->available_max_power += 5.0f;
		}
		else
		{
			//�߹���ģʽ������200W
			chassis_pwr_ctrl->available_max_power += 200.0f;
		}
	}
	//���ݵ�����ʱ��ʹ��
	else
	{
		chassis_pwr_ctrl->available_max_power += 0.0f;
	}
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//����Ԥ��ת��/************************ ����ط�Ҫ�� *************************/
		chassis_pwr_ctrl->pre_rpm[i] = ((fp32)chassis_move->chassis_3508[i].chassis_motor_measure->speed_rpm) / M3508_RR;

		//����ÿ�����ֵ�Ԥ�⹦��
		chassis_pwr_ctrl->pre_target_power[i] = chassis_pwr_ctrl->k2[i] * TORQUE_to_Icmd * chassis_move->chas_3508_pid[i].out + 
																						chassis_pwr_ctrl->k1[i] * powf(chassis_pwr_ctrl->pre_rpm[i], 2.0f) + 
																						TORQUE_to_Icmd / 9.55f * chassis_move->chas_3508_pid[i].out * chassis_pwr_ctrl->pre_rpm[i] + 
																						chassis_pwr_ctrl->constant[i];
		
		//������Ԥ�⹦��
		chassis_pwr_ctrl->pre_target_power_sum += chassis_pwr_ctrl->pre_target_power[i];
	}
	
	//�����Ԥ�⹦�ʳ������̵�ǰ���������, ��ô��3508���PID��out���б�������
	//ͬʱ��6020�����ֵ����ֵ
	if(chassis_pwr_ctrl->pre_target_power_sum >= chassis_pwr_ctrl->available_max_power)
	{
		for(uint8_t i = 0; i < 4; i ++)
		{
			chassis_move->chassis_3508[i].give_current = (int16_t)(chassis_move->chas_3508_pid[i].out * 
																										chassis_pwr_ctrl->available_max_power / 
																										chassis_pwr_ctrl->pre_target_power_sum);
			chassis_move->chassis_6020[i].give_current = (int16_t)(chassis_move->chas_6020_pid[i].out);
		}
	}
	//����������ֱ�Ӹ�ֵ
	else
	{
		for(uint8_t i = 0; i < 4; i ++)
		{
			chassis_move->chassis_3508[i].give_current = (int16_t)(chassis_move->chas_3508_pid[i].out);
			chassis_move->chassis_6020[i].give_current = (int16_t)(chassis_move->chas_6020_pid[i].out);
		}
	}
}
