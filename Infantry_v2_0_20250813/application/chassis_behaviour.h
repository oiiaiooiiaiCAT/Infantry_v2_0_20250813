/****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c
  * @brief      ����ң������ֵ������������Ϊ��
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
	CHASSIS_ZERO_FORCE, 									//��������״̬���൱�ڶϵ�
	CHASSIS_NO_MOVE,											//���̱��־�ֹ
	CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,		//����ģʽ�����̸�����̨yaw�Ƕ�
	CHASSIS_OPEN,													//����ģʽ��ң��ֱֵ��ת��ΪCAN���ߵ���
	CHASSIS_SPIN,													//��תģʽ
	
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10		//�� CHASSIS_OPEN ģʽ��, ң�������Ըñ������͵�can��


//����ң�����������õ�����Ϊģʽ
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
//���ݵ�ǰ��Ϊģʽ���õ��̿��Ʋ���
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

//ȫ�ֱ��������ڴ洢������Ϊģʽ
extern chassis_behaviour_e chassis_behaviour_mode;


#endif
