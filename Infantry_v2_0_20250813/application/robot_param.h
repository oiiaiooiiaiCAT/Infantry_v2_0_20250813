/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025     ���¾�              1. done
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

//��Ҫ�ļǵ�ע�͵�����Ҫ�ĳ�0

/*
*���ֶ��壺
 @param  	Hero_robot
					Infantry_robot
					Balance_robot
					Sentinel_robot
					Engineer_robot
*/

//Ӣ�ۺ궨��
//#define Hero_robot 1

//��ͨ�����궨��
#define Infantry_robot 1

////ƽ�ⲽ���궨��
//#define Balance_robot 1

////�ڱ��궨��
//#define Sentinel_robot 1

////���̺궨��
//#define Engineer_robot 1

/*
*�������ඨ�壺
 @param  	Mecanum wheel
					Omni_wheel
					Balance_wheel
					steering_wheel
*/

////���ֵ���
//#define Mecanum_wheel 1 

//ȫ���ֵ���
#define Omni_wheel 1

////���ֵ���
//#define steering_wheel 1

////ƽ���������
//#define Balance_wheel 1


/*
*��̨���ඨ�壺
 @param  ��û��ã�����Ƿ��
*/



////���綨��
//#define Super_cap 1

#endif
