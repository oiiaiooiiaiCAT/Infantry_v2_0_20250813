/****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c
  * @brief      ����ң������ֵ������������Ϊ��
  * @note       
  *			���Ҫ���һ���µ���Ϊģʽ
	*
  *			1.��.h�ļ��е� chassis_behaviour_eö�� ������µĵ���ģʽCHASSIS_XXX_XXX
  *			
  *			2.��.c�ļ��еĺ���������ĩβ����µĺ�������, �����ļ�ĩβ����µľ������ʵ�ֺ���
  *			
  *			3.��.c�ļ��е� chassis_behaviour_mode_set() �����е� ���ݼ������ı�ģʽ ������µ��ж�, 
	*				���� "//������Ϊģʽѡ��һ�����̿���ģʽ" ��������ж�
  *				(ע���� chassis_task.h �е� chassis_mode_eö�� ���ж��Ƿ�Ҫ�����ö��)
  *			
	*			4.��.c�ļ��е� chassis_behaviour_control_set() ������������ж�
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

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"


/* ���������� */
static void chassis_no_move_control											(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control	(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control								(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_spin_control												(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_zero_force_control									(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_open_set_control										(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


//������Ϊģʽ����, �ᱣ�浱ǰ������Ϊģʽ, ��ʼ��Ϊ����ģʽ
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          ����ң��������λ�úͼ����������õ�����Ϊģʽ, ��Ϊÿ����Ϊģʽѡ����ʵĵ��̿���ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL) return;

	//ң��������ģʽ
	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		//CHASSIS_ZERO_FORCE,  CHASSIS_NO_MOVE,  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
		//CHASSIS_NO_FOLLOW_YAW,  CHASSIS_OPEN,  CHASSIS_SPIN,
		//���ϲ�������ѡ��
		chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
	}
	else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}
	else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_SPIN;
	}
	
	/* ң�������µ������ݼ���x��c��shift���ı�ģʽ��ÿ2������ˢ��һ�Σ�����Ҫһֱ���Ų��ܱ�֤ģʽ��ȷ */
	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_ZERO_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}
	else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_RELATIVE_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
	}
	else if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) && (chassis_move_mode->chassis_RC->key.v & GIMBAL_SPIN_KEYBOARD))
	{
		chassis_behaviour_mode = CHASSIS_SPIN;
	}
	
	//����̨��ĳЩģʽ�£����ʼ���� ���̲���
	if (gimbal_cmd_to_chassis_stop())
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}

	//������Ϊģʽѡ��һ�����̿���ģʽ
	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
	}
	else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW ;
	}
	else if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)		//��ʵ�ַ�ʽ
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
	}
	else if (chassis_behaviour_mode == CHASSIS_OPEN)		//��ʵ�ַ�ʽ
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
	}
}

/**
  * @brief          ���ݵ�ǰ��Ϊģʽ���ö�Ӧ�Ŀ��ƺ��������õ����˶�����������
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector, ��������������Ϣ.
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;


	if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
	{
		chassis_no_move_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
	{
		chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if(chassis_behaviour_mode == CHASSIS_SPIN)
	{
		chassis_spin_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		chassis_zero_force_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_OPEN)
	{
		chassis_open_set_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
	}
}

/**
  * @brief          ���̲��ƶ�����Ϊ״̬����, ����ģʽ�ǲ�����Ƕ�
  * @author         RM
  * @param[in]      vx_set ǰ�����ٶ�, ��ֵ ǰ���ٶ�, 	��ֵ �����ٶ�
  * @param[in]      vy_set ���ҵ��ٶ�, ��ֵ �����ٶ�,   ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת���ٶ�, ��ֵ ��ʱ����ת, ��ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          ���̸�����̨����Ϊ״̬����, ����ģʽ�Ǹ�����̨�Ƕ�, ������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�, ��ֵ ǰ���ٶ�, ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�, ��ֵ �����ٶ�, ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
	
	//���� ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	
	//ֱ�����ý��ٶ�Ϊ0
	*wz_set = 0.0f;
}

/**
  * @brief          ���ù̶�����ת�ٶ�wz, �ڸ���ң������������vx��vy
  * @param[in]      vx_set ǰ�����ٶ�, ��ֵ ǰ���ٶ�,	  ��ֵ �����ٶ�
  * @param[in]      vy_set ���ҵ��ٶ�, ��ֵ �����ٶ�,	  ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶ�,	 ��ֵ ��ʱ����ת, ��ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
	
	//���� ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ 
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	
	//���ý��ٶ�Ϊ ��ת�ٶ� ���� ��������
	*wz_set = CHASSIS_SPIN_SPEED * CHASSIS_SPIN_FACTOR ;

	return;
}

/**
  * @brief          ������������Ϊ״̬����, ����ģʽ��raw, �趨ֵ��ֱ�ӷ��͵�can���������趨ֵ��Ϊ0
  * @author         RM
  * @param[in]      vx_set ǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set ���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set ��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          ֱ�ӽ� ң����ͨ��ֵ ���� ����ϵ�� �������CAN������
  * @param[in]      vx_set ǰ�����ٶ�, ��ֵ ǰ���ٶȣ�  ��ֵ �����ٶ�
  * @param[in]      vy_set ���ҵ��ٶȣ���ֵ �����ٶȣ�  ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ�  ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

	*vx_set =	 chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]  * CHASSIS_OPEN_RC_SCALE;
	*vy_set =	-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]  * CHASSIS_OPEN_RC_SCALE;
	*wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;

	return;
}
