/** @file       chassis_calculate.c
  * @brief      �����˶�ѧ��/�����
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

#include "chassis_calculate.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "arm_math.h"


/* ���������� */
const static fp32 wheel_angle_offset[4] = {(0.1f), (0.1f), (0.1f), (0.1f)};		//�����ֳ�ʼ�Ƕ�ƫ��ֵ, ��ʱ��Ϊ��
static fp32 relative_angle;			//��̨��Ե��̵ĽǶ�
static fp32 chassis_yaw;				//���̵�ǰƫ���Ƕ�
static fp32 inv_matrix[2][2];		//�������ת����
static fp32 for_matrix[2][2];		//��������ת����

/**
  * @brief          �˶�ѧ����ǰ��׼��
  * @param[out]     ��ȡ���̵�ǰƫ���Ƕȡ���̨��Ե��̵ĽǶ��Լ�������ת����
  * @retval         none
  */
static void pre_chas_cal(void)
{
	//��ȡ���̵�ǰƫ���Ƕ�
	chassis_yaw = rad_format(chassis_move.chassis_yaw);
	//��ȡ��̨��Ե��̵ĽǶ�
	relative_angle = rad_format(chassis_move.chassis_yaw_motor->relative_angle);
	//�����������ת����
	inv_matrix[0][0] = arm_cos_f32(relative_angle);	inv_matrix[0][1] = -arm_sin_f32(relative_angle);
	inv_matrix[1][0] = arm_sin_f32(relative_angle);	inv_matrix[1][1] =	arm_cos_f32(relative_angle);
	
	//��������ת���� Ϊ �������ת���� ��ת�þ���
	for_matrix[0][0] = inv_matrix[0][0];	for_matrix[0][1] = inv_matrix[1][0];
	for_matrix[1][0] = inv_matrix[0][1];	for_matrix[1][1] = inv_matrix[1][1];
}

/**
  * @brief          �˶�ѧ���������˿�����Ʋ���
  * @param[in&out]  wheel_angle		���ֵ�Ŀ��Ƕ�
  * @param[in&out]  wheel_speed		���ֵ�Ŀ���ٶ�
  * @retval         none
  */
static void smooth_control(fp32 *wheel_angle, fp32 *wheel_speed)
{
	//���嵱ǰ�Ķ��ֽǶȡ�����Ŀ��Ƕ��뵱ǰ�ǶȲ�
	fp32 current_angle[4], angle_delta[4];
	for(uint8_t i = 0; i < 4; i ++)
	{
		//��ȡ���ֵ�ǰ�ĽǶ�
		current_angle[i] = chassis_move.chassis_6020[i].angle;
		
		//�������Ŀ��Ƕ��뵱ǰ�ǶȲ�
		angle_delta[i] = rad_format(wheel_angle[i] - current_angle[i]);
		
		//�ͽ�תλ
		if((PI/2.0f < angle_delta[i]) && (angle_delta[i] < PI))
		{
			wheel_angle[i] -= PI;	wheel_speed[i] *= -wheel_speed[i];
		}
		else if((-PI < angle_delta[i]) && (angle_delta[i] < -PI/2.0f))
		{
			wheel_angle[i] += PI;	wheel_speed[i] *= -wheel_speed[i];
		}
	}
}

/**
  * @brief          �˶�ѧ�����
  * @param[in]			vx_set	�����̨���õ�x�����ٶȷ���
  * @param[in]			vy_set	�����̨���õ�y�����ٶȷ���
  * @param[in]			wz_set	������ת�ٶ�
  * @param[out]			wheel_angle		����Ŀ��Ƕ�ָ��
  * @param[out]			wheel_speed		����Ŀ���ٶ�ָ��
  * @retval         none
  */
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;
	
	//�˶�ѧ����ǰ��׼��
	pre_chas_cal();
	
	//������̵�ʵ���ٶ���������
	fp32 vx_actual, vy_actual;
	
	//��ת�任, ���ٶ���������̨����ϵת��Ϊ��������ϵ
	vx_actual = vx_set * inv_matrix[0][0] + vy_set * inv_matrix[0][1];
	vy_actual = vx_set * inv_matrix[1][0] + vy_set * inv_matrix[1][1];
	
	//��������ӻ�õĽ��ٶȷ��������ٶȷ���
	fp32 wz_x[4], wz_y[4], vx_total[4], vy_total[4];
	
	//�����ٶȴ� rad/s ת��Ϊ m/s
	wz_set *= MOTOR_TO_CENTER;
	
	//����ÿ�����ӻ�ȡ�Ľ��ٶȷ���(�ȼ�����ٶ�Ϊ��, �ٸ���ʵ������ж��Ƿ�ȡ��)
	wz_x[LF] =	wz_set * 0.70710678f;		wz_y[LF] = -wz_x[LF];
	wz_x[LB] =	wz_x[LF];								wz_y[LB] =	wz_x[LF];
	wz_x[RB] = -wz_x[LF];								wz_y[RB] =	wz_x[LF];
	wz_x[RF] = -wz_x[LF];								wz_y[RF] = -wz_x[LF];
	
	if(wz_set < 0){ for(uint8_t i = 0; i < 4; i ++){ wz_x[i] *= -1; wz_y[i] *= -1; } }
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//��������ӵ����ٶȷ���, 
		vx_total[i] = vx_actual + wz_x[i];
		vy_total[i] = vy_actual + wz_y[i];
		
		//Ŀ��Ƕ� = �ٶ�������Ե��̵ĽǶ� + ���̵�ǰƫ���Ƕ� - ���ֽǶ�ƫ��
		wheel_angle[i] = rad_format(atan2f(vy_total[i], vx_total[i]) + chassis_yaw - wheel_angle_offset[i]);
		//Ŀ���ٶ��ù��ɶ��������
		arm_sqrt_f32((powf(vx_total[i], 2.0f) + powf(vy_total[i], 2.0f)), &wheel_speed[i]);
		
		//˿������
		smooth_control(wheel_angle, wheel_speed);
	}
}

/**
  * @brief          �˶�ѧ������
  * @param[in]			wheel_angle		���ֵ�ǰ�Ƕ�ָ��
  * @param[in]			wheel_speed		���ֵ�ǰ�ٶ�ָ��
  * @param[out]			vx	�����̨�ĵ�ǰx�����ٶȷ���
  * @param[out]			vy	�����̨�ĵ�ǰy�����ٶȷ���
  * @param[out]			wz	���̵�ǰ����ת�ٶ�
  * @retval         none
  */
void chas_for_cal(fp32 *wheel_angle, fp32 *wheel_speed, fp32 *vx, fp32 *vy, fp32 *wz)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL)) return;
	
	//�˶�ѧ����ǰ��׼��
	pre_chas_cal();
	
	//��������ӵ����ٶȷ��������ٶȷ��������̵�ʵ���ٶ���������
	fp32 vx_total[4], vy_total[4], wz_x, wz_y, vx_actual, vy_actual;
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//��������ӵ����ٶȷ���
		vx_total[i] = wheel_speed[i] * arm_cos_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
		vy_total[i] = wheel_speed[i] * arm_sin_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
	}
	
	//�ⷽ�����vx_actual��vy_actual��wz_x (ע: �ⷽ��ʱ�ٶ����ٶ�Ϊ��ʱ�뷽��)
	vx_actual = ((vx_total[LF] + vx_total[RB]) / 2.0f + (vx_total[LB] + vx_total[RF]) / 2.0f) / 2.0f;
	vy_actual = ((vy_total[LF] + vy_total[RB]) / 2.0f + (vy_total[LB] + vy_total[RF]) / 2.0f) / 2.0f;
	wz_x = ((vx_total[LF] - vx_total[RB]) / 2.0f + (vx_total[LB] - vx_total[RF]) / 2.0f) / 2.0f;
	
	//��ת�任, ���ٶ������ɵ�������ϵת��Ϊ��̨����ϵ
//	*vx = vx_actual * for_matrix[0][0] + vy_actual * for_matrix[0][1];
//	*vy = vx_actual * for_matrix[1][0] + vy_actual * for_matrix[1][1];
	
	//��������ת��ת�任, ֱ�ӵõ���������ϵ�µ��ٶ�����, ���ڴ򻬿��Ƽ���
	*vx = vx_actual;
	*vy = vy_actual;
	
	//���ݽ��ٶȷ���������ٶ�,������� m/s ת��Ϊ rad/s
	*wz = wz_x * 1.41421356f / MOTOR_TO_CENTER;
}
