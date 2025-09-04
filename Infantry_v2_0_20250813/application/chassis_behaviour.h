/****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c
  * @brief      根据遥控器的值，决定底盘行为。
  * @note       
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

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H


#include "struct_typedef.h"
#include "chassis_task.h"

typedef enum
{
	CHASSIS_ZERO_FORCE, 									//底盘无力状态，相当于断电
	CHASSIS_NO_MOVE,											//底盘保持静止
	CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,		//步兵模式，底盘跟随云台yaw角度
	CHASSIS_OPEN,													//开环模式，遥控值直接转换为CAN总线电流
	CHASSIS_SPIN,													//旋转模式
	
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10		//在 CHASSIS_OPEN 模式下, 遥控器乘以该比例发送到can上


//根据遥控器输入设置底盘行为模式
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
//根据当前行为模式设置底盘控制参数
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

//全局变量，用于存储底盘行为模式
extern chassis_behaviour_e chassis_behaviour_mode;


#endif
