/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "remote_control.h"


/*������ݽṹ��*/
typedef struct
{
	uint16_t	ecd;						//��ǰ�Ƕ�
	int16_t		speed_rpm;      //��ǰת��
	int16_t		given_current;  //��ǰʵ��Ť�ص���
	uint8_t		temperate;      //��ǰ�¶�
	int16_t		last_ecd;       //��һ�εĽǶ�
	
} MOTOR_MEASURE_t;


#ifdef chassis_board

#define CHASSIS_3508_CAN hcan1  //2p
#define CHASSIS_6020_CAN hcan2  //4p

static uint8_t chassis_can_send_data[8];
static uint8_t gimbal_can_send_data[8];

/* CAN�߷�������յ�ID */
//CAN1�ٿغͽ��� 3508, CAN2�ٿغͽ��� 6020
typedef enum
{
	CAN1_CHAS_3508_ID = 0x200,
	CAN1_3508_M1_ID = 0x201,
	CAN1_3508_M2_ID = 0x202,
	CAN1_3508_M3_ID = 0x203,
	CAN1_3508_M4_ID = 0x204,
	
	CAN2_CHAS_6020_ID = 0x1FE,
	CAN2_6020_M1_ID = 0x205,
	CAN2_6020_M2_ID = 0x206,
	CAN2_6020_M3_ID = 0x207,
	CAN2_6020_M4_ID = 0x208,

} CHAS_ID_e;

extern void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_CHAS_6020(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
extern const MOTOR_MEASURE_t *get_chassis_motor_measure_point(uint8_t i);
#endif


#ifdef gimbal_board
#define GIMBAL_3508_CAN 	hcan1  //2p
#define GIMBAL_6020_CAN 	hcan2  //4p

/* CAN�߷�������յ�ID */
//CAN1�ٿغͽ��� 3508��2006��CAN2�ٿغͽ��� 6020
typedef enum
{
	CAN1_GIMB_3508_2006_ID = 0x200,
	CAN1_FRIC_M1_ID = 0x201,
	CAN1_FRIC_M2_ID = 0x202,
	CAN1_PLUCK_ID 	= 0x203,
	
	CAN2_GIMB_6020_ID = 0x1FE,
	CAN2_YAW_ID = 0x205,
	CAN2_PIT_ID = 0x206,
	
} GIMB_ID_e;

extern void CAN_cmd_GIMB_FP(int16_t fric1, int16_t fric2, int16_t pluck, int16_t rev);
extern void CAN_cmd_GIMB_6020(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern const MOTOR_MEASURE_t *get_pluck_motor_measure_point(void);
extern const MOTOR_MEASURE_t *get_yaw_gimbal_motor_measure_point(void);
extern const MOTOR_MEASURE_t *get_pitch_gimbal_motor_measure_point(void);
#endif // DEBUG

#endif
