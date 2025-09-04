/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.h
  * @brief      ���������� ͨ����������ʱ�����ж�.�ṩ ��⹳�Ӻ���,������ں���.
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
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//�������Լ���Ӧ�豸˳��
enum errorList
{
	DBUS_TOE = 0,
	CHASSIS_MOTOR1_TOE,
	CHASSIS_MOTOR2_TOE,
	CHASSIS_MOTOR3_TOE,
	CHASSIS_MOTOR4_TOE,
	CHASSIS_MOTOR5_TOE,
	CHASSIS_MOTOR6_TOE,
	CHASSIS_MOTOR7_TOE,
	CHASSIS_MOTOR8_TOE,
	PLUCK_MOTOR_TOE,
	YAW_GIMBAL_MOTOR_TOE,
	PITCH_GIMBAL_MOTOR_TOE,
	BOARD_GYRO_TOE,
	BOARD_ACCEL_TOE,
	BOARD_MAG_TOE,
	REFEREE_TOE,
	RM_IMU_TOE,
	OLED_TOE,
	
	ERROR_LIST_LENGHT,
};

typedef __packed struct
{
	uint32_t new_time;
	uint32_t last_time;
	uint32_t lost_time;
	uint32_t work_time;
	uint16_t set_offline_time : 12;
	uint16_t set_online_time : 12;
	uint8_t  enable : 1;
	uint8_t  priority : 4;
	uint8_t  error_exist : 1;
	uint8_t  is_lost : 1;
	uint8_t  data_is_error : 1;

	fp32     frequency;

	bool_t (*data_is_error_fun)(void);
	void   (*solve_lost_fun)(void);
	void   (*solve_data_error_fun)(void);
	
} error_t;


extern void detect_task(void const *pvParameters);
extern bool_t toe_is_error(uint8_t err);
extern void detect_hook(uint8_t toe);
extern const error_t *get_error_list_point(void);


#endif
