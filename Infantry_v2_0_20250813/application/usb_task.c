/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb���������Ϣ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "chassis_task.h"
#include "remote_control.h"
static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
	MX_USB_DEVICE_Init();
	error_list_usb_local = get_error_list_point();


	while(1)
	{
		osDelay(1000);
		usb_printf(
			"******************************\r\n\
			voltage percentage:%d%% \r\n\
			DBUS:%s\r\n\
			chassis motor1:%s\r\n\
			chassis motor2:%s\r\n\
			chassis motor3:%s\r\n\
			chassis motor4:%s\r\n\
			yaw motor:%s\r\n\
			pitch motor:%s\r\n\
			trigger motor:%s\r\n\
			gyro sensor:%s\r\n\
			accel sensor:%s\r\n\
			mag sensor:%s\r\n\
			referee usart:%s\r\n\
			gimbal_can_send_data:%s\r\n\
			chassis_can_send_data:%s\r\n\
			sbus_rx_data:%s\r\n\
			******************************\r\n",
			get_battery_percentage(), 
			status[error_list_usb_local[DBUS_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
			status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
			status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
			status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
			status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
			status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
			status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
			status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
			status[error_list_usb_local[REFEREE_TOE].error_exist],
			gimbal_can_send_data[0],
			chassis_can_send_data[0],
			sbus_rx_buf[0]
		);
	}
}

//void usb_task(void const * argument)
//{
//    MX_USB_DEVICE_Init();
//    error_list_usb_local = get_error_list_point();


//    while(1)
//    {
//        osDelay(1000);
//        usb_printf(
//"******************************\r\n\
//voltage percentage:%d%% \r\n\
//DBUS:%s\r\n\
//chassis motor1:%s\r\n\
//chassis motor2:%s\r\n\
//chassis motor3:%s\r\n\
//chassis motor4:%s\r\n\
//yaw motor:%s\r\n\
//pitch motor:%s\r\n\
//trigger motor:%s\r\n\
//gyro sensor:%s\r\n\
//accel sensor:%s\r\n\
//mag sensor:%s\r\n\
//referee usart:%s\r\n\
//			gimbal_can_send_data[0]:%c\r\n\
//			gimbal_can_send_data[1]:%c\r\n\
//			gimbal_can_send_data[2]:%c\r\n\
//			gimbal_can_send_data[3]:%c\r\n\
//			gimbal_can_send_data[4]:%c\r\n\
//			gimbal_can_send_data[5]:%c\r\n\
//			gimbal_can_send_data[6]:%c\r\n\
//			gimbal_can_send_data[7]:%c\r\n\
//			chassis_can_send_data[0]:%c\r\n\
//			chassis_can_send_data[1]:%c\r\n\
//			chassis_can_send_data[2]:%c\r\n\
//			chassis_can_send_data[3]:%c\r\n\
//			chassis_can_send_data[4]:%c\r\n\
//			chassis_can_send_data[5]:%c\r\n\
//			chassis_can_send_data[6]:%c\r\n\
//			chassis_can_send_data[7]:%c\r\n\
//******************************\r\n",
//            get_battery_percentage(), 
//            status[error_list_usb_local[DBUS_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//            status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//            status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//            status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//            status[error_list_usb_local[REFEREE_TOE].error_exist],
//						gimbal_can_send_data[0],
//						gimbal_can_send_data[1],
//						gimbal_can_send_data[2],
//						gimbal_can_send_data[3],
//						gimbal_can_send_data[4],
//						gimbal_can_send_data[5],
//						gimbal_can_send_data[6],
//						gimbal_can_send_data[7],
//						chassis_can_send_data[0],
//						chassis_can_send_data[1],
//						chassis_can_send_data[2],
//						chassis_can_send_data[3],
//						chassis_can_send_data[4],
//						chassis_can_send_data[5],
//						chassis_can_send_data[6],
//						chassis_can_send_data[7])

//						;

//    }

//}

//static void usb_printf(const char *fmt,...)
//{
//    static va_list ap;
//    uint16_t len = 0;

//    va_start(ap, fmt);

//    len = vsprintf((char *)usb_buf, fmt, ap);

//    va_end(ap);


//    CDC_Transmit_FS(usb_buf, len);
//}
void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}
