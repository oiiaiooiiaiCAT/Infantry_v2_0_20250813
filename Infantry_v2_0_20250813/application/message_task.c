#include "message_task.h"
#include "gimbal_task.h"
#include "chassis_power_control.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "auto_aim.h"


static void send_data_to_chassis(gimbal_control_t *feedback_update);
static void send_to_gimbal(void);


/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void message_task(void const *pvParameters)
{
    // 初始化延迟
    vTaskDelay(MESSAGE_TASK_INIT_TIME * 5);

    while(1) {
      #ifdef gimbal_board
        send_data_to_chassis(&gimbal_control);
				send_to_minipc();
				vTaskDelay(MESSAGE_TASK_INIT_TIME);  // 每次循环的延迟 
      #endif // DEBUG
			
      #ifdef chassis_board
        send_to_gimbal();
				vTaskDelay(MESSAGE_TASK_INIT_TIME * 2);
      #endif // DEBUG
      
    }
}

static void send_to_gimbal(void){
//发送陀螺仪数据给云台
static uint8_t send_to_gimbal[15];
  fp32_to_bytes yaw, roll, pitch;
  yaw.fp32 = chassis_move.chassis_yaw;
  pitch.fp32 = chassis_move.chassis_pitch;
  roll.fp32 = SPIN_SPEED;

  send_to_gimbal[0] = 's';

  send_to_gimbal[1] = yaw.bytes[0];
  send_to_gimbal[2] = yaw.bytes[1];
  send_to_gimbal[3] = yaw.bytes[2];
  send_to_gimbal[4] = yaw.bytes[3];
  //
  send_to_gimbal[5] = pitch.bytes[0];
  send_to_gimbal[6] = pitch.bytes[1];
  send_to_gimbal[7] = pitch.bytes[2];
  send_to_gimbal[8] = pitch.bytes[3];
  //底盘旋转速度
  send_to_gimbal[9] = roll.bytes[0];
  send_to_gimbal[10] = roll.bytes[1];
  send_to_gimbal[11] = roll.bytes[2];
  send_to_gimbal[12] = roll.bytes[3];
	send_to_gimbal[13] = robot_id;
	
  send_to_gimbal[14] = 'e';
  
  usart1_tx_dma_enable(send_to_gimbal, 15 );

}

static void send_data_to_chassis(gimbal_control_t *feedback_update) {
  //0xfe为帧头，0xfd为帧尾
    static uint8_t data_chassis[15];//传到云台的数据
    fp32* INS_yaw_add; //存储INS指针
    fp32 INS_yaw, MOTOR_yaw; //暂存数据
    INS_yaw_add = get_INS_angle_point(); //获取INS指针
    INS_yaw = *(INS_yaw_add+INS_GYRO_X_ADDRESS_OFFSET);//将INS的yaw值传出来，方便后续操作
    MOTOR_yaw = feedback_update->gimbal_yaw_motor.relative_angle;
    data_chassis[0] = 0XFE;
  
    uint32_t *ptr = (uint32_t *)&INS_yaw;//类型转换
    // 提取每个字节
    data_chassis[1] = (*ptr >> 0) & 0xFF;   // 最低字节
    data_chassis[2] = (*ptr >> 8) & 0xFF;   // 第二个字节
    data_chassis[3] = (*ptr >> 16) & 0xFF;  // 第三个字节
    data_chassis[4] = (*ptr >> 24) & 0xFF;  // 最高字节
  
    uint32_t *ptr_motor = (uint32_t *)&MOTOR_yaw;//类型转换
    // 提取每个字节
    data_chassis[5] = (*ptr_motor >> 0) & 0xFF;   // 最低字节
    data_chassis[6] = (*ptr_motor >> 8) & 0xFF;   // 第二个字节
    data_chassis[7] = (*ptr_motor >> 16) & 0xFF;  // 第三个字节
    data_chassis[8] = (*ptr_motor >> 24) & 0xFF;  // 最高字节
		
		data_chassis[9] = 0;//闲置字节
		data_chassis[10] = 0;
		data_chassis[11] = 0;
		data_chassis[12] = 0;
		 data_chassis[13] = 0X00; //
    data_chassis[14] = 0XFD;  // 最高字节
    usart1_tx_dma_enable(data_chassis, 15);//DMA发送数据
}

