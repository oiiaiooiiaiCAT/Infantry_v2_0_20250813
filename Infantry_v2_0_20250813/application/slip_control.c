/**
  * @file       slip_control.h
  * @brief     	�򻬿���
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
	
#include "slip_control.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include <math.h>

/************************************
����ά����Ҫ�����Ĳ���

1.�������˲�����(����Ҫ)
	��������Э���� Q_data[9] ���� ��Ҫ����ʵ��ϵͳ����
	�۲�����Э���� R_kf_data[9] ���� ��Ҫ���ݴ��������ȵ���

2.������ֵ����chassis_slip_threshold ���� ��Ҫʵ�ʲ��Ե���

3.PID����pid_params[3] ���� KP��KI��KD


���鰴����˳�����:

�۲� predicted_speed �Ƿ����;
���� Q �� R ����ʹ�˲�ƽ��;
�۲� diffs ֵ�仯, ���� chassis_slip_threshold;
����PID;
************************************/

/* �������˲���ʵ�� */
KalmanFilter_t chassis_speed_kalman;

/* ���ж����� */
fp32 chassis_slip_threshold = 0.10f;

/* PID������ʵ�� */
pid_type_def slip_pid;

/* ARM��ѧ�����ʵ�� */
arm_matrix_instance_f32 R;      // ��ת����

arm_matrix_instance_f32 H;      // �۲����
arm_matrix_instance_f32 F;      // ״̬ת�ƾ���
arm_matrix_instance_f32 B;      // ���ƾ���
arm_matrix_instance_f32 Q;      // ��������Э����
arm_matrix_instance_f32 R_kf;   // �۲�����Э����
arm_matrix_instance_f32 P;      // �������Э����

/* ��̬�������� */
static void quaternion_to_rotation_matrix(const fp32* q, arm_matrix_instance_f32 *R);
static void init_arm_matrices(void);
static fp32 calc_time_diff(uint32_t new_tick, uint32_t old_tick);

//pid����
fp32 pid_params[3]={0.8f, 0.0f, 0.05f};

/**
  * @brief ��ʼ���ٶ�Ԥ���˲���
  */
void init_speed_predictor(void)
{
	static uint8_t init_flag = 0;
	if(!init_flag) {
		/* ��ʼ��ARM���� */
		init_arm_matrices();
		
		/* ��ʼ���������˲��� */
		Kalman_Filter_Init(&chassis_speed_kalman, 3, 3, 3);
		
		/* ���þ���ָ�� */
		chassis_speed_kalman.F_data = F.pData;
		chassis_speed_kalman.B_data = B.pData;
		chassis_speed_kalman.H_data = H.pData;
		chassis_speed_kalman.Q_data = Q.pData;
		chassis_speed_kalman.R_data = R_kf.pData;
		chassis_speed_kalman.P_data = P.pData;
		
		init_flag = 1;
		PID_init(&slip_pid, PID_USUAL, pid_params, 0.3f, 0.05f); // �����������ͻ������޷�
	}
}

/**
  * @brief ��ȡ�ںϺ�ĵ����ٶ�
  */
void get_INS_speed(fp32* vx, fp32* vy, fp32* wz)
{
    static uint32_t last_tick = 0;
    static fp32 integrated_vx = 0.0f;
    static fp32 integrated_vy = 0.0f;
    
    /* ʱ������ */
    uint32_t current_tick = osKernelSysTick();
    fp32 dt = calc_time_diff(current_tick, last_tick);
    last_tick = current_tick;

    /* ��ȡ���������� */
    const fp32* gyro = get_gyro_data_point();
    const fp32* accel = get_accel_data_point();
    const fp32* quat = get_INS_quat_point();
    
    /* ������ת���� */
    quaternion_to_rotation_matrix(quat, &R);
    
    /* ������������ */
    float32_t gravity_world[3] = {0, 0, 9.81f};
    float32_t gravity_body[3] = {0};
    arm_matrix_instance_f32 gravity_world_mat = {3, 1, gravity_world};
    arm_matrix_instance_f32 gravity_body_mat = {3, 1, gravity_body};
    
    arm_mat_mult_f32(&R, &gravity_world_mat, &gravity_body_mat);

    /* �������Լ��ٶ� */
    float32_t linear_accel[3] = {
        accel[0] - gravity_body[0],
        accel[1] - gravity_body[1],
        accel[2] - gravity_body[2]
    };
    
    /* �ٶȻ��� */
    integrated_vx += linear_accel[0] * dt;
    integrated_vy += linear_accel[1] * dt;
    
    /* ������ */
    *vx = integrated_vx;
    *vy = integrated_vy;
    *wz = gyro[INS_GYRO_Z_ADDRESS_OFFSET];
}

/* ������� */
fp32 vx_wheel, vy_wheel, wz_wheel;
fp32 vx_ins, vy_ins, wz_ins;
fp32 predicted_speed[3];
float32_t pid_output_x, pid_output_y, pid_output_z;
float32_t diffs[3];	 //���Ƽ��

/**
  * @brief ���ƿ���������, ���ڼ���Ƿ���, ����vx_set��vy_set��wz_set�����趨��
  */
void slip_control(chassis_move_t* chassis)
{
	/* ��ʼ����� */
	static uint8_t init_flag = 0;
	if(!init_flag) {
		init_speed_predictor();
		init_flag = 1;
	}
	
	get_INS_speed(&vx_ins, &vy_ins, &wz_ins);
	
	//��ȡת�����ٶ�, ͨ�� chassis_feedback_update() ������� chas_for_cal() ����ʵ��
	vx_wheel = chassis->vx;
	vy_wheel = chassis->vy;
	wz_wheel = chassis->wz;
	
	/* ������Ԥ�� */
//  float32_t u[3] = {chassis->vx_set * 0.1f, chassis->vy_set * 0.1f, chassis->wz_set * 0.1f};
	float32_t u[3] = {chassis->vx_set, chassis->vy_set, chassis->wz_set};
	float32_t z[3] = {vx_wheel, vy_wheel, wz_wheel};
	
	arm_copy_f32(u, chassis_speed_kalman.ControlVector, 3);
	arm_copy_f32(z, chassis_speed_kalman.MeasuredVector, 3);
	
	Kalman_Filter_Update(&chassis_speed_kalman);
	arm_copy_f32(chassis_speed_kalman.xhat_data, predicted_speed, 3);
	
	arm_sub_f32(predicted_speed, (float32_t[]){vx_ins, vy_ins, wz_ins}, diffs, 3);
	arm_abs_f32(diffs, diffs, 3);
		
	/* �ֱ�������Ļ��Ʋ���Ӧ */
	for(int i = 0; i < 3; ++i)
	{
		if(diffs[i] > chassis_slip_threshold)
		{
			switch(i)
			{
				case 0: // X�Ử�ƴ���
					pid_output_x = - PID_calc(&slip_pid, predicted_speed[0], chassis->vx_set);
					chassis->vx_set *= pid_output_x;
					break;
				case 1: // Y�Ử�ƴ���
					pid_output_y = - PID_calc(&slip_pid, predicted_speed[1], chassis->vy_set);
					chassis->vy_set *= pid_output_y;
					break;
				case 2: // Z�Ử�ƴ���
					pid_output_z = - PID_calc(&slip_pid, predicted_speed[2], chassis->wz_set);
					chassis->wz_set *= pid_output_z;
					break;
			}
		}
	}
}

/******************** ARM��ѧ�⸨������ ********************/

/**
  * @brief          ����Ԫ��ת��Ϊ��ת����
	* @param[in]	    q ��Ԫ��
	* @param[out]     R	arm_matrix_instance_f32 �ṹ��ָ��
  * @retval         none
  */
static void quaternion_to_rotation_matrix(const fp32* q, arm_matrix_instance_f32 *R)
{
	/* ��Ԫ��ת��ת���� */
	float32_t q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
	
	float32_t* pData = R->pData;
	pData[0] = 1 - 2*q2*q2 - 2*q3*q3;
	pData[1] = 2*q1*q2 - 2*q0*q3;
	pData[2] = 2*q1*q3 + 2*q0*q2;
	
	pData[3] = 2*q1*q2 + 2*q0*q3;
	pData[4] = 1 - 2*q1*q1 - 2*q3*q3;
	pData[5] = 2*q2*q3 - 2*q0*q1;
	
	pData[6] = 2*q1*q3 - 2*q0*q2;
	pData[7] = 2*q2*q3 + 2*q0*q1;
	pData[8] = 1 - 2*q1*q1 - 2*q2*q2;
}

/**
  * @brief          Ϊÿ��������侲̬�ڴ�, ��ʼ��ÿ���������ֵ, ���� arm_mat_init_f32() �󶨾���ṹ����������
  */
static void init_arm_matrices(void)
{
	/* Ԥ��������ڴ� */
	static float32_t R_data[9] = {0};
	static float32_t F_data[9] = {1,0,0, 0,1,0, 0,0,1};
	static float32_t B_data[9] = {1,0,0, 0,1,0, 0,0,1};
	static float32_t H_data[9] = {1,0,0, 0,1,0, 0,0,1};
//  static float32_t Q_data[9] = {0.01,0,0, 0,0.01,0, 0,0,0.01};
	static float32_t Q_data[9] = {0.04,0,0, 0,0.04,0, 0,0,0.04};
//  static float32_t R_kf_data[9] = {0.1,0,0, 0,0.1,0, 0,0,0.1};
	static float32_t R_kf_data[9] = {0.4,0,0, 0,0.4,0, 0,0,0.4};
	static float32_t P_data[9] = {1,0,0, 0,1,0, 0,0,1};
	
	arm_mat_init_f32(&R, 3, 3, R_data);
	arm_mat_init_f32(&F, 3, 3, F_data);
	arm_mat_init_f32(&B, 3, 3, B_data);
	arm_mat_init_f32(&H, 3, 3, H_data);
	arm_mat_init_f32(&Q, 3, 3, Q_data);
	arm_mat_init_f32(&R_kf, 3, 3, R_kf_data);
	arm_mat_init_f32(&P, 3, 3, P_data);
}

/**
  * @brief          ȷ���������˲���ʱ���� dt
  */
static fp32 calc_time_diff(uint32_t new_tick, uint32_t old_tick)
{
	const uint32_t TICK_MAX = 0xFFFFFFFF;
	
	uint32_t diff = (new_tick >= old_tick) ? 
									(new_tick -  old_tick) :
									(TICK_MAX -  old_tick + new_tick);
	
	return diff * (1.0f / 168000000.0f); // 168mHzʱ��
}
