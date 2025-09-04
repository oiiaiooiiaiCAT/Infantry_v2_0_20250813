/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.h
  * @brief     	���̿�������
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
	
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "INS_task.h"

#define CHASSIS_TASK_INIT_TIME 357	//����ʼ����һ��ʱ��

#define CHASSIS_X_CHANNEL			1		//ǰ���ң����ͨ������
#define CHASSIS_Y_CHANNEL			0		//���ҵ�ң����ͨ������
#define CHASSIS_WZ_CHANNEL		2		//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_MODE_CHANNEL 	0		//ѡ�����״̬ ����ͨ����

//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f

//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ҡ������
#define CHASSIS_RC_DEADLINE 25

#define CHASSIS_CONTROL_TIME_MS 		2				//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME				0.002f	//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_FREQUENCE		500.0f	//�����������Ƶ��

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY		KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY 		KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY 		KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY		KEY_PRESSED_OFFSET_D

#define MAX_WHEEL_SPEED							3.0f	//�������̵������ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X	2.5f	//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y	2.5f	//�����˶��������ƽ���ٶ�

/* ����6020��3508���can���͵���ֵ */
#define MAX_6020_CAN_CURRENT 16000.0f
#define MAX_3508_CAN_CURRENT 16000.0f

/* ���ֽ����õ������� */
#define M3508_RR						19.20320855f				//M3508������ٱ�(M3508 Reduction Ratio)��3591 / 187
#define GM6020_Angle_Ratio	1303.63813886f			//�Ѽ�����Ļ���Radianת��ΪGM6020����Ƕ�Angle��ϵ����8191 / (2 * PI)
#define Wheel_Radius				0.0567f							//���ֵİ뾶����λ����
#define Wheel_Perimeter			0.35625661f					//�����ܳ���2.0f * PI * Wheel_Radius����λ����
#define MPS_to_RPM					3234.16461897f			//��M3508���ٶ�m/s -> ת��r/min��60.0f * M3508_RR / Wheel_Perimeter
#define RPM_to_Icmd					1.55125592f					//��M3508�����ת��RPM -> ���Ƶ���Icmd(�����ڿ���)��(187 /3591) * (16384 / 550)

/* ����3508���PID */	/***************����***************/
#define M3505_MOTOR_SPEED_PID_KP 				15000.0f
#define M3505_MOTOR_SPEED_PID_KI 				10.0f
#define M3505_MOTOR_SPEED_PID_KD				0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 	16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT	2000.0f

/* ����6020���PID */	/***************����***************/
#define GM6020_MOTOR_ANGLE_PID_KP 				15000.0f
#define GM6020_MOTOR_ANGLE_PID_KI 				10.0f
#define GM6020_MOTOR_ANGLE_PID_KD					0.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_OUT 		16000.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_IOUT		2000.0f

/* ������ת����PID */	/***************����***************/
#define CHASSIS_FOLLOW_GIMBAL_PID_KP		10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI		0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD		0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT		6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT	0.2f

//������תʱ��roll���ٶ�
#define CHASSIS_SPIN_SPEED 		SPIN_SPEED
#define CHASSIS_SPIN_FACTOR 	SPIN_FACTOR

//������ת�ٶ����ű���
#define CHASSIS_WZ_SET_SCALE 0.1f

#ifndef myabs
	#define myabs(x) (((x) >= 0) ? (x) : (-(x)))
#endif

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̻������̨��ԽǶ�
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //��������ת�ٶȿ���
  CHASSIS_VECTOR_RAW,                 //���Ƶ���ֱֵ�ӷ��͵�CAN������

} chassis_mode_e;

typedef struct
{
  const MOTOR_MEASURE_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
	fp32 angle_set;
  int16_t give_current;
	
} chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;              	//����ʹ�õ�ң����ָ��
	const gimbal_motor_t *chassis_yaw_motor;  	//����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
	const gimbal_motor_t *chassis_pitch_motor;	//����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
	const fp32 *chassis_INS_angle;            	//ȡ�����ǽ������ŷ����ָ��
	chassis_mode_e chassis_mode;              	//���̿���״̬��
	chassis_mode_e last_chassis_mode;         	//�����ϴο���״̬��
	chassis_motor_t chassis_3508[4];						//����3508�������
	chassis_motor_t chassis_6020[4];						//����6020�������
	pid_type_def chas_3508_pid[4];         			//����3508����ٶ�pid
	pid_type_def chas_6020_pid[4];         			//����6020����Ƕ�pid
	pid_type_def chassis_angle_pid;           	//���̸���Ƕ�pid
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;  	//ʹ��һ�׵�ͨ�˲������趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_vy;  	//ʹ��һ�׵�ͨ�˲������趨ֵ
//	first_order_filter_type_t chassis_cmd_slow_set_vz; 	//ʹ��һ�׵�ͨ�˲������趨ֵ
	
	fp32 vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
	fp32 wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 vx_set;											//�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy_set;											//�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;											//�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 chassis_relative_angle;			//��������̨����ԽǶȣ���λ rad
	fp32 chassis_relative_angle_set;	//���������̨���ƽǶ�����
	fp32 chassis_yaw_set;							//����ƫ��������
	
	fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
	fp32 vx_min_speed;  //���˷�������ٶ� ��λm/s
	fp32 vy_max_speed;  //��������ٶ� ��λm/s
	fp32 vy_min_speed;  //�ҷ�������ٶ� ��λm/s
	fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
	fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
	fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
	
} chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

//�����˶�����
extern chassis_move_t chassis_move;
extern fp32 yaw_set;


#endif
