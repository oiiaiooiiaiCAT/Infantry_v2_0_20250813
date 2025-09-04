/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c
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
	
#include "arm_math.h"
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "chassis_behaviour.h"
#include "chassis_calculate.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "INS_task.h"
#include "iwdg.h"
#include "pid.h"
#include "remote_control.h"
#include "slip_control.h"

#define rc_deadband_limit(input, output, dealine)	  \
{                                                		\
	if ((input) > (dealine) || (input) < -(dealine))	\
	{                                             		\
		(output) = (input);                          		\
	}                                             		\
	else                                          		\
	{                                             		\
		(output) = 0;                          					\
	}                                          				\
}

#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t chassis_high_water;
#endif


/* �����˶����� */
chassis_move_t chassis_move;


/**
  * @brief          ���̲������ݸ��£�����3508����ٶȡ�6020����Ƕȡ�ŷ���Ƕȡ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL) return;
	
	//���������������ڼ�¼���ֵ�ǰ�ǶȺ��ٶ�
	fp32 wheel_angle[4], wheel_speed[4];
	
	for (uint8_t i = 0; i < 4; i++)
	{
		//����3508����ٶȺͼ��ٶȣ����ٶ����ٶȵ�PID΢��
		chassis_move_update->chassis_3508[i].speed = chassis_move_update->chassis_3508[i].chassis_motor_measure->speed_rpm / MPS_to_RPM;
		wheel_speed[i] = chassis_move_update->chassis_3508[i].speed;
		chassis_move_update->chassis_3508[i].accel = chassis_move_update->chas_3508_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		//����6020����Ƕ�
		chassis_move_update->chassis_6020[i].angle = rad_format(chassis_move_update->chassis_6020[i].chassis_motor_measure->ecd / GM6020_Angle_Ratio);
		wheel_angle[i] = chassis_move_update->chassis_6020[i].angle;
	}
	
	//������������
	chassis_move_update->chassis_yaw 	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET	 ));
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
	chassis_move_update->chassis_roll	 = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET ));
		
	//���˶�ѧ�������µ��������ٶ� x, ƽ���ٶ� y, ��ת�ٶ� wz, ����ϵΪ����ϵ(ǰx��y��z)
	chas_for_cal(wheel_angle, wheel_speed, &chassis_move_update->vx, &chassis_move_update->vy, &chassis_move_update->wz);
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ��, ң����ָ���ʼ��, 3508���̵��ָ���ʼ��, ��̨�����ʼ��, �����ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL) return;

	//����3508�ٶȻ�pidֵ
	const static fp32 chas_3508_pid_param[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//����6020�ǶȻ�pidֵ
	const static fp32 chas_6020_pid_param[3] = {GM6020_MOTOR_ANGLE_PID_KP, GM6020_MOTOR_ANGLE_PID_KI, GM6020_MOTOR_ANGLE_PID_KD};
	//���̽Ƕ�pidֵ
	const static fp32 chassis_yaw_pid_param[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
	
	//���̿���״̬Ϊԭʼ
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	//��ȡң����ָ��
	chassis_move_init->chassis_RC = get_remote_control_point();
	//��ȡ��������̬��ָ��
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	//��ȡ��̨�������ָ��
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	
	//��ȡ���̵������ָ�룬��ʼ��PID 
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_move_init->chassis_3508[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move_init->chas_3508_pid[i], PID_USUAL, chas_3508_pid_param, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		chassis_move_init->chassis_6020[i].chassis_motor_measure = get_chassis_motor_measure_point(i + 4);
		PID_init(&chassis_move_init->chas_6020_pid[i], PID_USUAL, chas_6020_pid_param, GM6020_MOTOR_ANGLE_PID_MAX_OUT, GM6020_MOTOR_ANGLE_PID_MAX_IOUT);
	}
	//��ʼ���Ƕ�PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_USUAL, chassis_yaw_pid_param, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	
	//��һ���˲�����б����������
	const static fp32 chassis_x_order_filter = CHASSIS_ACCEL_X_NUM;
	const static fp32 chassis_y_order_filter = CHASSIS_ACCEL_Y_NUM;
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, &chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, &chassis_y_order_filter);

	//��� ��С�ٶ�
	chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//����һ������
	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ��ң������������õ��̿���ģʽ
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL) return;
	chassis_behaviour_mode_set(chassis_move_mode);	//in file "chassis_behaviour.c"
}

/**
  * @brief          ����ģʽ�ı�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if (chassis_move_transit == NULL) return;

	if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) return;
	
	//���������̨ģʽ
	if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
	//���벻������̨ģʽ
	else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
	}
	
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) return;
	
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * ( CHASSIS_VX_RC_SEN);
	vy_set_channel = vy_channel * (-CHASSIS_VY_RC_SEN);
	
	//���̿���
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	}
	
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
	}

	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	
	//ң����������
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}
	
	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;	
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ�� chassis_behaviour_control_set �������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL) return;

	fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

	//��ȡ������������ֵ
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	wz_set = - wz_set;
	//������̨ģʽ
	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		//������תPID���ٶ�
		chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
		//�ٶ��޷�
		chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	//������ת
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		//��wz_set������ת�ٶ�����
		chassis_move_control->wz_set = wz_set;
		//�ٶ��޷�
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	//��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
	{
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = wz_set;
		chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
		chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
	}
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	fp32 wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	
	//���̶����˶�ѧ�����
	chas_inv_cal(chassis_move_control_loop->vx_set,
	             chassis_move_control_loop->vy_set,
	             chassis_move_control_loop->wz_set,
							 wheel_angle, wheel_speed);
	
	//������Ķ���Ŀ��ǶȺ�Ŀ���ٶȸ�ֵ��angle_set��speed_set
	for(uint8_t i = 0; i < 4; i ++)
	{
		chassis_move_control_loop->chassis_6020[i].angle_set = wheel_angle[i];
		chassis_move_control_loop->chassis_3508[i].speed_set = wheel_speed[i];
	}
	
	//raw����ֱ�ӷ��أ������ƶ��ֽǶȵĵ�����Ҫ��PID����
	if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			chassis_move_control_loop->chassis_3508[i].give_current = (int16_t)(wheel_speed[i] * MPS_to_RPM * RPM_to_Icmd);
			PID_calc(&chassis_move_control_loop->chas_6020_pid[i], chassis_move_control_loop->chassis_6020[i].angle, wheel_angle[i]);
			chassis_move_control_loop->chassis_6020[i].give_current = (int16_t)(chassis_move_control_loop->chas_6020_pid[i].out);
		}
		return;
	}
	
	//������
	slip_control(chassis_move_control_loop);

	for(uint8_t i = 0; i < 4; i++)
	{
		//����pid
		PID_calc(&chassis_move_control_loop->chas_3508_pid[i], chassis_move_control_loop->chassis_3508[i].speed, chassis_move_control_loop->chassis_3508[i].speed_set);
		PID_calc(&chassis_move_control_loop->chas_6020_pid[i], chassis_move_control_loop->chassis_6020[i].angle, chassis_move_control_loop->chassis_6020[i].angle_set);
	}
	
	//���ʿ���(�� chassis_power_control() �����и�����3508��6020�����ֵ����ֵ)
	chassis_power_control(chassis_move_control_loop);
}

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
	//HAL_IWDG_Refresh(&hiwdg);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);	//����һ��ʱ��
	//HAL_IWDG_Refresh(&hiwdg);
	chassis_init(&chassis_move); //���̳�ʼ��
	
	//�жϵ��̵���Ƿ�����
	while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || \
				 toe_is_error(CHASSIS_MOTOR5_TOE) || toe_is_error(CHASSIS_MOTOR6_TOE) || toe_is_error(CHASSIS_MOTOR7_TOE) || toe_is_error(CHASSIS_MOTOR8_TOE) || \
				 toe_is_error(DBUS_TOE))
	{ vTaskDelay(CHASSIS_CONTROL_TIME_MS); }
	
	while (1)
	{
		//���õ��̿���ģʽ
		chassis_set_mode(&chassis_move);
		//ģʽ�л����ݱ���
		chassis_mode_change_control_transit(&chassis_move);
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		//���̿���������
		chassis_set_contorl(&chassis_move);
		//���̿���PID����
		chassis_control_loop(&chassis_move);

		//ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
		if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE) && \
					toe_is_error(CHASSIS_MOTOR5_TOE) && toe_is_error(CHASSIS_MOTOR6_TOE) && toe_is_error(CHASSIS_MOTOR7_TOE) && toe_is_error(CHASSIS_MOTOR8_TOE)))
		{
			//��ң�������ߵ�ʱ�򣬷��͸����̵�������.
			if (toe_is_error(DBUS_TOE))
			{
				CAN_cmd_CHAS_6020(0, 0, 0, 0);
				CAN_cmd_CHAS_3508(0, 0, 0, 0);
			}
			else
			{						
				//���Ϳ��Ƶ���
				CAN_cmd_CHAS_6020(chassis_move.chassis_6020[0].give_current, chassis_move.chassis_6020[1].give_current,
													chassis_move.chassis_6020[2].give_current, chassis_move.chassis_6020[3].give_current);
				CAN_cmd_CHAS_3508(chassis_move.chassis_3508[0].give_current, chassis_move.chassis_3508[1].give_current,
													chassis_move.chassis_3508[2].give_current, chassis_move.chassis_3508[3].give_current);
				HAL_IWDG_Refresh(&hiwdg);
			}
		}
		//ϵͳ��ʱ
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

		#if INCLUDE_uxTaskGetStackHighWaterMark
			chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}
