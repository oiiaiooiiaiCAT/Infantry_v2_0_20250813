/** @file       chassis_calculate.c
  * @brief      底盘运动学正/逆解算
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


/* 变量声明区 */
const static fp32 wheel_angle_offset[4] = {(0.1f), (0.1f), (0.1f), (0.1f)};		//各舵轮初始角度偏移值, 逆时针为正
static fp32 relative_angle;			//云台相对底盘的角度
static fp32 chassis_yaw;				//底盘当前偏航角度
static fp32 inv_matrix[2][2];		//逆解算旋转矩阵
static fp32 for_matrix[2][2];		//正解算旋转矩阵

/**
  * @brief          运动学解算前的准备
  * @param[out]     获取底盘当前偏航角度、云台相对底盘的角度以及计算旋转矩阵
  * @retval         none
  */
static void pre_chas_cal(void)
{
	//获取底盘当前偏航角度
	chassis_yaw = rad_format(chassis_move.chassis_yaw);
	//获取云台相对底盘的角度
	relative_angle = rad_format(chassis_move.chassis_yaw_motor->relative_angle);
	//计算逆解算旋转矩阵
	inv_matrix[0][0] = arm_cos_f32(relative_angle);	inv_matrix[0][1] = -arm_sin_f32(relative_angle);
	inv_matrix[1][0] = arm_sin_f32(relative_angle);	inv_matrix[1][1] =	arm_cos_f32(relative_angle);
	
	//正解算旋转矩阵 为 逆解算旋转矩阵 的转置矩阵
	for_matrix[0][0] = inv_matrix[0][0];	for_matrix[0][1] = inv_matrix[1][0];
	for_matrix[1][0] = inv_matrix[0][1];	for_matrix[1][1] = inv_matrix[1][1];
}

/**
  * @brief          运动学逆解算纵享丝滑控制策略
  * @param[in&out]  wheel_angle		舵轮的目标角度
  * @param[in&out]  wheel_speed		舵轮的目标速度
  * @retval         none
  */
static void smooth_control(fp32 *wheel_angle, fp32 *wheel_speed)
{
	//定义当前的舵轮角度、舵轮目标角度与当前角度差
	fp32 current_angle[4], angle_delta[4];
	for(uint8_t i = 0; i < 4; i ++)
	{
		//获取舵轮当前的角度
		current_angle[i] = chassis_move.chassis_6020[i].angle;
		
		//计算舵轮目标角度与当前角度差
		angle_delta[i] = rad_format(wheel_angle[i] - current_angle[i]);
		
		//就近转位
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
  * @brief          运动学逆解算
  * @param[in]			vx_set	相对云台设置的x方向速度分量
  * @param[in]			vy_set	相对云台设置的y方向速度分量
  * @param[in]			wz_set	底盘旋转速度
  * @param[out]			wheel_angle		舵轮目标角度指针
  * @param[out]			wheel_speed		舵轮目标速度指针
  * @retval         none
  */
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;
	
	//运动学解算前的准备
	pre_chas_cal();
	
	//定义底盘的实际速度向量分量
	fp32 vx_actual, vy_actual;
	
	//旋转变换, 将速度向量由云台坐标系转换为底盘坐标系
	vx_actual = vx_set * inv_matrix[0][0] + vy_set * inv_matrix[0][1];
	vy_actual = vx_set * inv_matrix[1][0] + vy_set * inv_matrix[1][1];
	
	//定义各轮子获得的角速度分量、总速度分量
	fp32 wz_x[4], wz_y[4], vx_total[4], vy_total[4];
	
	//将角速度从 rad/s 转换为 m/s
	wz_set *= MOTOR_TO_CENTER;
	
	//计算每个轮子获取的角速度分量(先假设角速度为正, 再根据实际情况判断是否取反)
	wz_x[LF] =	wz_set * 0.70710678f;		wz_y[LF] = -wz_x[LF];
	wz_x[LB] =	wz_x[LF];								wz_y[LB] =	wz_x[LF];
	wz_x[RB] = -wz_x[LF];								wz_y[RB] =	wz_x[LF];
	wz_x[RF] = -wz_x[LF];								wz_y[RF] = -wz_x[LF];
	
	if(wz_set < 0){ for(uint8_t i = 0; i < 4; i ++){ wz_x[i] *= -1; wz_y[i] *= -1; } }
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//计算各轮子的总速度分量, 
		vx_total[i] = vx_actual + wz_x[i];
		vy_total[i] = vy_actual + wz_y[i];
		
		//目标角度 = 速度向量相对底盘的角度 + 底盘当前偏航角度 - 舵轮角度偏移
		wheel_angle[i] = rad_format(atan2f(vy_total[i], vx_total[i]) + chassis_yaw - wheel_angle_offset[i]);
		//目标速度用勾股定理即可求出
		arm_sqrt_f32((powf(vx_total[i], 2.0f) + powf(vy_total[i], 2.0f)), &wheel_speed[i]);
		
		//丝滑控制
		smooth_control(wheel_angle, wheel_speed);
	}
}

/**
  * @brief          运动学正解算
  * @param[in]			wheel_angle		舵轮当前角度指针
  * @param[in]			wheel_speed		舵轮当前速度指针
  * @param[out]			vx	相对云台的当前x方向速度分量
  * @param[out]			vy	相对云台的当前y方向速度分量
  * @param[out]			wz	底盘当前的旋转速度
  * @retval         none
  */
void chas_for_cal(fp32 *wheel_angle, fp32 *wheel_speed, fp32 *vx, fp32 *vy, fp32 *wz)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL)) return;
	
	//运动学解算前的准备
	pre_chas_cal();
	
	//定义各轮子的总速度分量、角速度分量、底盘的实际速度向量分量
	fp32 vx_total[4], vy_total[4], wz_x, wz_y, vx_actual, vy_actual;
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		//计算各轮子的总速度分量
		vx_total[i] = wheel_speed[i] * arm_cos_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
		vy_total[i] = wheel_speed[i] * arm_sin_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
	}
	
	//解方程求出vx_actual、vy_actual、wz_x (注: 解方程时假定角速度为逆时针方向)
	vx_actual = ((vx_total[LF] + vx_total[RB]) / 2.0f + (vx_total[LB] + vx_total[RF]) / 2.0f) / 2.0f;
	vy_actual = ((vy_total[LF] + vy_total[RB]) / 2.0f + (vy_total[LB] + vy_total[RF]) / 2.0f) / 2.0f;
	wz_x = ((vx_total[LF] - vx_total[RB]) / 2.0f + (vx_total[LB] - vx_total[RF]) / 2.0f) / 2.0f;
	
	//旋转变换, 将速度向量由底盘坐标系转换为云台坐标系
//	*vx = vx_actual * for_matrix[0][0] + vy_actual * for_matrix[0][1];
//	*vy = vx_actual * for_matrix[1][0] + vy_actual * for_matrix[1][1];
	
	//不进行旋转旋转变换, 直接得到底盘坐标系下的速度向量, 用于打滑控制计算
	*vx = vx_actual;
	*vy = vy_actual;
	
	//根据角速度分量计算角速度,并将其从 m/s 转换为 rad/s
	*wz = wz_x * 1.41421356f / MOTOR_TO_CENTER;
}
