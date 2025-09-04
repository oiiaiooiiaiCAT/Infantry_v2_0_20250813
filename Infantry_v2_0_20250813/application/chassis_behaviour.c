/****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c
  * @brief      根据遥控器的值，决定底盘行为。
  * @note       
  *			如果要添加一个新的行为模式
	*
  *			1.在.h文件中的 chassis_behaviour_e枚举 下添加新的底盘模式CHASSIS_XXX_XXX
  *			
  *			2.在.c文件中的函数声明区末尾添加新的函数声明, 并在文件末尾添加新的具体控制实现函数
  *			
  *			3.在.c文件中的 chassis_behaviour_mode_set() 函数中的 根据键盘来改变模式 下添加新的判断, 
	*				并在 "//根据行为模式选择一个底盘控制模式" 下添加新判断
  *				(注意在 chassis_task.h 中的 chassis_mode_e枚举 下判断是否要添加新枚举)
  *			
	*			4.在.c文件中的 chassis_behaviour_control_set() 函数中添加新判断
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


/* 函数声明区 */
static void chassis_no_move_control											(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_infantry_follow_gimbal_yaw_control	(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control								(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_spin_control												(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_zero_force_control									(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_open_set_control										(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


//底盘行为模式变量, 会保存当前底盘行为模式, 初始化为无力模式
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          根据遥控器开关位置和键盘输入设置底盘行为模式, 并为每种行为模式选择合适的底盘控制模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL) return;

	//遥控器设置模式
	if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		//CHASSIS_ZERO_FORCE,  CHASSIS_NO_MOVE,  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
		//CHASSIS_NO_FOLLOW_YAW,  CHASSIS_OPEN,  CHASSIS_SPIN,
		//以上参数均可选择
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
	
	/* 遥控器在下档，根据键盘x、c、shift来改变模式，每2个毫秒刷新一次，所以要一直按着才能保证模式正确 */
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
	
	//当云台在某些模式下，像初始化， 底盘不动
	if (gimbal_cmd_to_chassis_stop())
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}

	//根据行为模式选择一个底盘控制模式
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
	else if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)		//无实现方式
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
	}
	else if (chassis_behaviour_mode == CHASSIS_OPEN)		//无实现方式
	{
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
	}
}

/**
  * @brief          根据当前行为模式调用对应的控制函数来设置底盘运动的三个参数
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector, 包括底盘所有信息.
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
  * @brief          底盘不移动的行为状态机下, 底盘模式是不跟随角度
  * @author         RM
  * @param[in]      vx_set 前进的速度, 正值 前进速度, 	负值 后退速度
  * @param[in]      vy_set 左右的速度, 正值 左移速度,   负值 右移速度
  * @param[in]      wz_set 旋转的速度, 正值 逆时针旋转, 负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘跟随云台的行为状态机下, 底盘模式是跟随云台角度, 底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度, 正值 前进速度, 负值 后退速度
  * @param[in]      vy_set左右的速度, 正值 左移速度, 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
	
	//根据 遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	
	//直接设置角速度为0
	*wz_set = 0.0f;
}

/**
  * @brief          设置固定的旋转速度wz, 在根据遥控器输入设置vx和vy
  * @param[in]      vx_set 前进的速度, 正值 前进速度,	  负值 后退速度
  * @param[in]      vy_set 左右的速度, 正值 左移速度,	  负值 右移速度
  * @param[in]      wz_set 旋转速度,	 正值 逆时针旋转, 负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */
static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
	
	//根据 遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值 
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	
	//设置角速度为 旋转速度 乘以 比例因子
	*wz_set = CHASSIS_SPIN_SPEED * CHASSIS_SPIN_FACTOR ;

	return;
}

/**
  * @brief          底盘无力的行为状态机下, 底盘模式是raw, 设定值会直接发送到can总线上且设定值都为0
  * @author         RM
  * @param[in]      vx_set 前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set 左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set 旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          直接将 遥控器通道值 乘以 比例系数 后输出到CAN总线上
  * @param[in]      vx_set 前进的速度, 正值 前进速度，  负值 后退速度
  * @param[in]      vy_set 左右的速度，正值 左移速度，  负值 右移速度
  * @param[in]      wz_set 旋转速度，  正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
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
