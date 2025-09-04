/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#define FEEDFORWARD_GAIN 0.00f  // ǰ������
//pitch speed close-loop PID params, max out and max iout
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        2900.0f
#define PITCH_SPEED_PID_KI        80.0f
#define PITCH_SPEED_PID_KD        00.0f
//#define PITCH_SPEED_PID_KP        1000.0f
//#define PITCH_SPEED_PID_KI        0.0f
//#define PITCH_SPEED_PID_KD        7.5f

#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f


//yaw speed close-loop PID params, max out and max iout
//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f

#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
//#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
//#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
//#define YAW_GYRO_ABSOLUTE_PID_KD        0.7f
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.1f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   20.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

// //pitch encode angle close-loop PID params, max out and max iout
// //pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
// #define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
// #define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
// #define PITCH_ENCODE_RELATIVE_PID_KD 0.5f
// //pitch encode angle close-loop PID params, max out and max iout
 //pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
//����������ǰ�ĵ��ɡ�������ɿ���
#define PITCH_ENCODE_RELATIVE_PID_KP 25.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.3f
//pitch encode angle close-loop PID params, max out and max iout

// //pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
// #define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
// #define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
// #define PITCH_ENCODE_RELATIVE_PID_KD 0.3f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        28.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.5f

#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

#define GIMBAL_ANGLE_Z_RC_SEN 0.000002f
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 200
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
#define WZ_CHANNEL 2
//turn 180��
//��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//��ͷ��̨�ٶ�
#define TURN_SPEED    0.04f
//���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
//ԭ����10�����ǻ��ǲ�����ȫ����
#define RC_DEADBAND   10

//��̨С���ݵ�ʱ����Ĳ���,������
#define GIMBAL_SPIN_SPEED SPIN_SPEED
#define GIMBAL_SPIN_FACTOR SPIN_FACTOR
#define GIMBAL_SPIN_PPL -1270

#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 2

//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  0
#define YAW_TURN    0

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
	#define MOTOR_ECD_TO_RAD 0.000766990394f	//2*  PI  /8192
#endif


//�������ֵ���� 0��8191
#define ecd_format(ecd)   \
{                         \
	if ((ecd) > ECD_RANGE)  \
		(ecd) -= ECD_RANGE; 	\
	else if ((ecd) < 0)     \
		(ecd) += ECD_RANGE; 	\
}

#define gimbal_total_pid_clear(gimbal_clear)                                             \
{                                                                                        \
	gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
	gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
	PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
																																												 \
	gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
	gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
	PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
}

		
typedef enum
{
	GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
	GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
	GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
	
} gimbal_motor_mode_e;

typedef struct
{
	fp32 kp;
	fp32 ki;
	fp32 kd;

	fp32 set;
	fp32 get;
	fp32 err;

	fp32 max_out;
	fp32 max_iout;

	fp32 Pout;
	fp32 Iout;
	fp32 Dout;

	fp32 out;
	
} gimbal_PID_t;

typedef struct
{
	const MOTOR_MEASURE_t *gimbal_motor_measure;
	gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_relative_angle_pid;
	pid_type_def gimbal_motor_gyro_pid;
	gimbal_motor_mode_e gimbal_motor_mode;
	gimbal_motor_mode_e last_gimbal_motor_mode;
	uint16_t offset_ecd;
	fp32 max_relative_angle ; //rad
	fp32 min_relative_angle; //rad

	fp32 relative_angle;     //rad
	fp32 relative_angle_set; //rad
	fp32 absolute_angle;     //rad
	fp32 absolute_angle_set; //rad
	fp32 motor_gyro;         //rad/s
	fp32 motor_gyro_set;
	fp32 motor_speed;
	fp32 raw_cmd_current;
	fp32 current_set;
	int16_t given_current;

} gimbal_motor_t;

typedef struct
{
//	fp32 max_yaw = 3.0;
//	fp32 min_yaw = -3.0;
//	fp32 max_pitch = 0.261750996;
//	fp32 min_pitch = 0.82528168;
//	uint16_t max_yaw_ecd = 0x0DE0;
//	uint16_t min_yaw_ecd = 0x1CE5;
//	uint16_t max_pitch_ecd = 0x19B7;
//	uint16_t min_pitch_ecd = 0x1C36;
	fp32 max_yaw;
	fp32 min_yaw;
	fp32 max_pitch;
	fp32 min_pitch;
	uint16_t max_yaw_ecd;
	uint16_t min_yaw_ecd;
	uint16_t max_pitch_ecd;
	uint16_t min_pitch_ecd;
	uint8_t step;
	
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;
extern gimbal_control_t gimbal_control;

extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
//����̨�����ڹ̶���ֵ
extern fp32 yaw_target ;
extern fp32 yaw_curret ;
extern uint8_t auto_aim_flag ;
extern fp32 aim_flag;
extern pid_type_def AIM;


extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);
extern void gimbal_task(void const *pvParameters);
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
extern void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
extern void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);


#endif
