#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H
#include "main.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "bsp_usart.h"
#include "message_task.h"
#include "shoot.h"
#define PI_RAD PI/180
//�����ǽ��Ӿ��ĵ�λ���꡾0��1���ֵ�360�ȣ�Ȼ��ת���ɻ��ȣ��ٳ���Ƶ�ʷ�ֹ�ػ��ӳٵ��¹��䣬����1*180*PI/180/100
//#define AUTO_AIM_PPL 1.0f/15.0f
#define AUTO_AIM_PPL 1.0f/5.0f
#define yaw_before 0.2f
#define pitch_before 0.2f

#define noise 0.15f
//�ж�һ�ε���Ҫ��ʱ��
#define IRQHandler_TIME 0.000014f 
//����ʱ
#define DELAY_TIME -1.0f/CONTINUE_TRIGGER_SPEED
//���Ĵ�С
#define PACKET_SIZE 16

#define FILTER_WINDOW_SIZE 10  // �˲����ڴ�С���ɸ�����Ҫ����

#define FILTER_ORDER 5  // �˲�������
#define SAMPLE_RATE 300  // ������ (Hz)
#define CUTOFF_FREQ 5  // ��ֹƵ�� (Hz)

// �������壬�������뱣��һ��
#define START_BYTE 's'
#define END_BYTE 'e'
#define SCALE_YAW_PITCH (100.0f / (32768 - 1))  // ��������һ��
#define SCALE_DISTANCE (1000.0f / (32768 - 1))  // ��������һ��

typedef struct {
    int16_t yaw;          // ƫ����
    int16_t pitch;        // ������
    int16_t distance;     // ����
    int16_t shoot_delay; // ����ӳ�
} ReceivedData;

extern fp32 yaw_set;
extern fp32 yaw_target;

extern void auto_aim(uint8_t* data_send_from_pc);
extern void send_to_minipc(void);
#endif
