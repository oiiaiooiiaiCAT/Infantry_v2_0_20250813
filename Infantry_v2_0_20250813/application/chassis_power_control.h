/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.h
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
	
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "chassis_task.h"
#include "main.h"

#define TORQUE_to_Icmd 1.907e-5f	//计算中将力矩torque转换为控制电流Icmd的系数

extern uint8_t robot_id ;

//功率限制结构体
typedef struct{
	float k1[4], k2[4], constant[4];	//功率模型参数
	
	fp32 max_power_limit;					//默认最大功率限制
	fp32 available_max_power;				//底盘当前可用最大功率
	fp32 current_power_buffer;				//当前缓冲能量剩余值
	fp32 pre_target_power[4];				//每个舵轮的预测功率
	fp32 pre_target_power_sum;				//总预测功率
	int16_t pre_rpm[4];						//预测转速
	
}	CHASSIS_POWER_CONTROL_t;

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_move);


#endif
