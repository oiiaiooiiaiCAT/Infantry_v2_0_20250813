/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c
  * @brief      底盘功率控制
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
 * @brief		功率控制函数初始化，用于给功率模型系数赋值
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
	* @brief					功率控制，可实现为每个舵轮轮向电机分配最大可用功率
  * @param[in]      chassis_pwr_ctrl: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_move)
{
	//只会初始化一遍，确定功率模型的系数
	if(!Init_Done) Chassis_PowerCtrl_Init(model_coefficients);
		
	//从裁判系统获取最大功率限制并获取当前缓冲能量剩余值
	get_chassis_power_and_buffer(&chassis_pwr_ctrl->max_power_limit, &chassis_pwr_ctrl->current_power_buffer);
	
	//判断缓冲能量剩余值是否大于10
	if(chassis_pwr_ctrl->current_power_buffer <= 10.0f) chassis_pwr_ctrl->current_power_buffer = 0.0f;
	
	//先基于缓冲能量计算底盘当前可用最大功率，为每个舵向电机预留4W的功率
	chassis_pwr_ctrl->available_max_power = chassis_pwr_ctrl->max_power_limit + chassis_pwr_ctrl->current_power_buffer - 16;
	
	//通过遥控器按键Q/E切换超级电容状态
	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
	{
		cap_info.cap_status = 0;	//关闭超级电容
	}
	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
	{
		cap_info.cap_status = 1;	//开启超级电容
	}
	
	//再基于超级电容计算底盘当前可用最大功率
	//如果电容电量超过5%
	if (cap_info.energy_percent > 5)
	{
		if (cap_info.cap_status == 0)
		{
			//正常模式，略高于最大可用功率5W
			chassis_pwr_ctrl->available_max_power += 5.0f;
		}
		else
		{
			//高功率模式，增加200W
			chassis_pwr_ctrl->available_max_power += 200.0f;
		}
	}
	//电容电量低时不使用
	else
	{
		chassis_pwr_ctrl->available_max_power += 0.0f;
	}
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//计算预测转速/************************ 这个地方要改 *************************/
		chassis_pwr_ctrl->pre_rpm[i] = ((fp32)chassis_move->chassis_3508[i].chassis_motor_measure->speed_rpm) / M3508_RR;

		//计算每个舵轮的预测功率
		chassis_pwr_ctrl->pre_target_power[i] = chassis_pwr_ctrl->k2[i] * TORQUE_to_Icmd * chassis_move->chas_3508_pid[i].out + 
																						chassis_pwr_ctrl->k1[i] * powf(chassis_pwr_ctrl->pre_rpm[i], 2.0f) + 
																						TORQUE_to_Icmd / 9.55f * chassis_move->chas_3508_pid[i].out * chassis_pwr_ctrl->pre_rpm[i] + 
																						chassis_pwr_ctrl->constant[i];
		
		//计算总预测功率
		chassis_pwr_ctrl->pre_target_power_sum += chassis_pwr_ctrl->pre_target_power[i];
	}
	
	//如果总预测功率超过底盘当前可用最大功率, 那么对3508电机PID的out进行比例缩放
	//同时给6020电机赋值电流值
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
	//不超功率则直接赋值
	else
	{
		for(uint8_t i = 0; i < 4; i ++)
		{
			chassis_move->chassis_3508[i].give_current = (int16_t)(chassis_move->chas_3508_pid[i].out);
			chassis_move->chassis_6020[i].give_current = (int16_t)(chassis_move->chas_6020_pid[i].out);
		}
	}
}
