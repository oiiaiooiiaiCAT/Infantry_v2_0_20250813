/*****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.h
  * @brief      У׼�豸��������̨,������,���ٶȼ�,������,����.��̨У׼����Ҫ�������
  *             �������С��ԽǶ�.��̨У׼����Ҫ������Ư.���ٶȼƺʹ�����У׼��û��ʵ��
  *             ��Ϊ���ٶȼƻ�û�б�ҪȥУ׼,�������ƻ�û����.����У׼��ʹM3508�������
  *             ����IDģʽ.
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


#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

//��imu��У׼,�����������÷�Ƶ��ǿ��
#define imu_start_buzzer()     buzzer_on(95, 10000)    
//����̨��У׼,�����������÷�Ƶ��ǿ��
#define gimbal_start_buzzer()  buzzer_on(31, 19999)
//�رշ�����
#define cali_buzzer_off()      buzzer_off()

//��ȡstm32Ƭ���¶ȣ�����imu�Ŀ����¶�
#define cali_get_mcu_temperature()	get_temprate()      

#define cali_flash_read(address, buf, len) 	flash_read((address), (buf), (len))										//��ȡ����
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))   //д�뺯��
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))            //��������


#define get_remote_ctrl_point_cali()        get_remote_control_point()		//��ȡң����ָ��
#define gyro_cali_disable_control()         RC_unable()                 	//��imu��У׼ʱ��,ʧ��ң����
#define gyro_cali_enable_control()          RC_restart(SBUS_RX_BUF_NUM)		//ʹ��ң����

//������������Ư
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//������INS task�ڵ���������Ư
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //�����flashҳ��ַ

/* �Ƚ���Ҫ�ĳ��� */
#define GYRO_CONST_MAX_TEMP   			45.0f  		//��������ǿ����¶�
#define GYRO_CALIBRATE_TIME   			20000  		//������У׼ʱ��
#define CALIBRATE_END_TIME    			20000			//У׼��ʱʱ��
#define RC_CALI_BUZZER_MIDDLE_TIME  10000			//У׼ʱ��ֻʣʮ��ʱ, ��������Ƶ����
#define RC_CALI_BUZZER_START_TIME   		0			//����ʼУ׼��ʱ��, ��������Ƶ����

#define RCCALI_BUZZER_CYCLE_TIME    400
#define RC_CALI_BUZZER_PAUSE_TIME   200
#define RC_CALI_VALUE_HOLE          600

#define CALI_FUNC_CMD_ON        1                   //����У׼
#define CALI_FUNC_CMD_INIT      0                   //�Ѿ�У׼��������У׼ֵ

#define CALIBRATE_CONTROL_TIME  1                   //1msϵͳ��ʱ

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //Ӳ���汾
#define CALIED_FLAG             0x55                //�����У׼�ı�־

#define rc_cali_buzzer_middle_on()  gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()   imu_start_buzzer()
#define RC_CMD_LONG_TIME            2000    


//У׼�豸����
typedef enum {
	CALI_HEAD = 0,    // ͷ���豸
	CALI_GIMBAL = 1,  // ��̨
	CALI_GYRO = 2,    // ������
	CALI_ACC = 3,     // ���ٶȼ�
	CALI_MAG = 4,     // ������
	//add more...
	CALI_LIST_LENGHT, // �豸����
} cali_id_e;

//У׼�豸����ṹ��
typedef __packed struct {
	uint8_t name[3];     	// �豸����(3�ֽ�)
	uint8_t cali_done;   	// У׼��־(0x55��ʾ��У׼)
	uint8_t flash_len :7;	// ���ݳ���(��4�ֽ�Ϊ��λ)
	uint8_t cali_cmd :1; 	// У׼����(1��ʾ��ҪУ׼)
	uint32_t *flash_buf; 	// ָ��У׼���ݵ�ָ��
	bool_t (*cali_hook)(uint32_t *data_ptr, bool_t cmd); // У׼���Ӻ���
} cali_sensor_t;

//ͷ��
typedef __packed struct {
	uint8_t self_id;           // ����ID
	uint16_t firmware_version; // �̼��汾
	int8_t temperature;        // IMU�����¶�
	fp32 latitude;             // γ��
	//ע:"temperature" �� "latitude"��Ӧ����head_cali_t, ��Ϊ���봴��һ���µ��豸�ͷ�����
} head_cali_t;

//��̨
typedef struct {
	uint16_t yaw_offset;      // ƫ�������ƫ��
	uint16_t pitch_offset;    // ���������ƫ��
	fp32 yaw_max_angle;       // ƫ�������Ƕ�
	fp32 yaw_min_angle;       // ƫ������С�Ƕ�
	fp32 pitch_max_angle;     // ���������Ƕ�
	fp32 pitch_min_angle;     // ��������С�Ƕ�
} gimbal_cali_t;

//�����ǡ����ٶȼơ�������
typedef struct {
    fp32 offset[3]; // x,y,z��ƫ��
    fp32 scale[3];  // x,y,z�����
} imu_cali_t;


extern void cali_param_init(void);
extern int8_t get_control_temperature(void);
extern void get_flash_latitude(float *latitude);
extern void calibrate_task(void const *pvParameters);


#endif
