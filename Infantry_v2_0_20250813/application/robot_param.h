/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025     滕勇军              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "struct_typedef.h"

//不要的记得注释掉，不要改成0

/*
*兵种定义：
 @param  	Hero_robot
					Infantry_robot
					Balance_robot
					Sentinel_robot
					Engineer_robot
*/

//英雄宏定义
//#define Hero_robot 1

//普通步兵宏定义
#define Infantry_robot 1

////平衡步兵宏定义
//#define Balance_robot 1

////哨兵宏定义
//#define Sentinel_robot 1

////工程宏定义
//#define Engineer_robot 1

/*
*底盘种类定义：
 @param  	Mecanum wheel
					Omni_wheel
					Balance_wheel
					steering_wheel
*/

////麦轮底盘
//#define Mecanum_wheel 1 

//全向轮底盘
#define Omni_wheel 1

////舵轮底盘
//#define steering_wheel 1

////平衡轮足底盘
//#define Balance_wheel 1


/*
*云台种类定义：
 @param  还没想好，就先欠着
*/



////超电定义
//#define Super_cap 1

#endif
