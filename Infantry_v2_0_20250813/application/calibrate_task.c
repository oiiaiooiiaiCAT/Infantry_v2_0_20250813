/*****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c
  * @brief      У׼�豸��������̨,������,���ٶȼ�,������,����.
	*							��̨У׼����Ҫ������Ư�������С��ԽǶ�.
	*							���ٶȼƺʹ�����У׼��û��ʵ��,��Ϊ���ٶȼƻ�û�б�ҪȥУ׼.
	*							�����ƻ�û����.
	*							����У׼��ʹM3508�����������IDģʽ.
	*
  * @note       ��һ����ң�����������ض�����
	*							
	*							�ڶ���������ҡ�˴��\../��״������2��
	*							
	*							��������ҡ�˴��./\.: ��ʼ������У׼
	*							        
	*							        ҡ�˴��'\/': ��ʼ��̨У׼
	*							        
	*							        ҡ�˴��/''\: ��ʼ����У׼
	*
  *             ������flash�У�����У׼���ݺ����� name[3] �� У׼��־λ cali_flag
  *             ����head_cali�а˸��ֽ�,������Ҫ12�ֽ���flash,�������0x080A0000��ʼ
  *             0x080A0000-0x080A0007: head_cali����
  *             0x080A0008: ����name[0]
  *             0x080A0009: ����name[1]
  *             0x080A000A: ����name[2]
  *             0x080A000B: У׼��־λ cali_flag,��У׼��־λΪ0x55,��ζ��head_cali�Ѿ�У׼��
	*
  *             *������豸*
  *             1.��.h�ļ���cali_id_eö���е�//add more...����������豸������
	*
  *             2.��.h�ļ�������µ����ݽṹ��(������Ϊxxx_cali_t),���ȱ�����4�ֽڱ���
	*
  *             3.��.c�ļ��е� "FLASH_WRITE_BUF_LENGHT"�����"sizeof(xxx_cali_t)"
	*							
	*							4.��.c�ļ�������µ�У׼���Ӻ���bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), �ǵ����Ϸ�����������������һ��
	*							
	*							5.��.c�ļ��е�"cali_name[CALI_LIST_LENGHT][3]"���������
  *             
	*							6.��.c�ļ��б�������������ӱ��� xxx_cali_t xxx_cail
	*
	*							7.��cali_sensor_buf[CALI_LIST_LENGHT]���У׼���ݵ�ַ
	*
	*							8.��cali_sensor_size[CALI_LIST_LENGHT]���У׼���ݵĳ���
	*
	*							9.��void *cali_hook_fun[CALI_LIST_LENGHT]ĩβ��Ӻ���ָ��
	*
	*							10.��RC_cmd_to_calibrate()�������������жϺ�ִ�����(�ؼ�)
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

#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "iwdg.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "can_receive.h"
#include "remote_control.h"
#include "INS_task.h"
#include "gimbal_task.h"


/* ���������� */
static void RC_cmd_to_calibrate(void);
static void cali_data_read(void);
static void cali_data_write(void);
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd);


#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t calibrate_task_stack;
#endif

/* ���������� */
static const RC_ctrl_t *calibrate_RC;   //ң�����ṹ��ָ�볣��
static head_cali_t     head_cali;       //ͷ��У׼���ݽṹ�����(��Ա����ϵͳID���̼��汾���¶ȡ�γ��)
static gimbal_cali_t   gimbal_cali;     //��̨У׼���ݽṹ�����
static imu_cali_t      accel_cali;      //���ٶȼ�У׼���ݽṹ�����
static imu_cali_t      gyro_cali;       //������У׼���ݽṹ�����
static imu_cali_t      mag_cali;        //������У׼���ݽṹ�����

/* ÿ���豸��Flash��ռ�õ��ܿռ�Ϊ: У׼���ݳ��� + 4�ֽ�(3�ֽ����� + 1�ֽڱ�־λ) */
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)

//��ʱ�洢���д�д�� Flash ��У׼����(�����豸У׼���ݡ��豸���ơ�У׼��־λ��)
static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

//У׼�豸�������飬���ڹ���������ҪУ׼���豸
cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

//�豸���ƶ��գ�����ÿ���豸����д����(3�ֽڹ̶�����)������ Flash �洢ʱ��ʶ�豸
static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//У׼���ݵ�ַ���飬���ڴ洢ÿ���豸У׼���ݵĵ�ַ
static uint32_t *cali_data_addr_buf[CALI_LIST_LENGHT] = {
	(uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
	(uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
	(uint32_t *)&mag_cali};

//�洢ÿ��У׼���ݽṹ�ĳ��ȣ���λ��4�ֽڳ���
//��head_cali_tΪ���ӣ��䳤��Ϊ��8�ֽ�/4�ֽ� = 2
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
	sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
	sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//����һ������ָ�����飬ÿ��Ԫ��ָ���Ӧ�豸��У׼����
//ʹ�� void * ��Ϊ�˼��ݲ�ͬ���͵ĺ���ָ��
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL,};


/**
  * @brief          У׼������ʼ��
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
	uint8_t i = 0;

	for (i = 0; i < CALI_LIST_LENGHT; i++)
	{
		cali_sensor[i].flash_len = cali_sensor_size[i];
		cali_sensor[i].flash_buf = cali_data_addr_buf[i];
		cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
	}

	cali_data_read();
	
	//��ϵͳ����ʱ, ���豸֮ǰ��У׼��, �����֮ǰ��У׼���ݽ��г�ʼ��, ȷ���豸����������
	for (i = 0; i < CALI_LIST_LENGHT; i++)
	{
		if (cali_sensor[i].cali_done == CALIED_FLAG)
		{
			if (cali_sensor[i].cali_hook != NULL)
			{
				cali_sensor[i].cali_hook(cali_data_addr_buf[i], CALI_FUNC_CMD_INIT);
			}
		}
	}
}

/**
  * @brief          ��ȡimu�����¶�, ��λ��
  * @param[in]      none
  * @retval         imu�����¶�
  */
int8_t get_control_temperature(void)
{
	return head_cali.temperature;
}

/**
  * @brief          ��ȡγ��,Ĭ��22.0f
  * @param[out]     latitude:fp32ָ�� 
  * @retval         none
  */
void get_flash_latitude(float *latitude)
{
	if (latitude == NULL)
	{
		return;
	}
	if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
	{
		*latitude = head_cali.latitude;
	}
	else
	{
		*latitude = 22.0f;
	}
}

/**
  * @brief          ʹ��ң������ʼУ׼�����������ǡ���̨������
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void)
{
	static const uint8_t BEGIN_FLAG   = 1;
	static const uint8_t GIMBAL_FLAG  = 2;
	static const uint8_t GYRO_FLAG    = 3;
	
	static uint32_t calibrate_systemTick;		//���ڼ�¼У׼�����е�ʱ���(��λ��ϵͳ�δ�ͨ���Ǻ���)
	static uint32_t rc_cmd_systemTick = 0;	//��¼У׼��ʼ��ϵͳʱ�䣨���ڳ�ʱ�жϣ�
	static uint16_t buzzer_time       = 0;	//���������м�ʱ��������ʾУ׼״̬��
	static uint16_t rc_cmd_time       = 0;	//��¼ҡ�˱����ض�λ�õ�ʱ��
	static uint8_t  rc_action_flag    = 0;	//У׼״̬������ǵ�ǰУ׼�׶�

	//����Ѿ���У׼���ͷ���
	for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++)
	{
		if (cali_sensor[i].cali_cmd)
		{
			buzzer_time = 0;
			rc_cmd_time = 0;
			rc_action_flag = 0;
			
			return;
		}
	}
	
	if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
	{
		//����У׼
		rc_cmd_systemTick = xTaskGetTickCount();	//��¼��ʼʱ��
		rc_action_flag = BEGIN_FLAG;							//����׼��״̬
		rc_cmd_time = 0;													//����ң����λ�ü�ʱ
	}
	else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
	{
		//��̨У׼
		rc_action_flag = 0;
		rc_cmd_time = 0;
		cali_sensor[CALI_GIMBAL].cali_cmd = 1;
		cali_buzzer_off();
	}
	else if (rc_action_flag == GYRO_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
	{
		//������У׼
		rc_action_flag = 0;
		rc_cmd_time = 0;
		cali_sensor[CALI_GYRO].cali_cmd = 1;
		//����IMU�¶�
		head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
		//�¶��޷�
		if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
		{
			head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
		}
		cali_buzzer_off();
	}
	
	/* ң����λ���ж� */
	if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_mid(calibrate_RC->rc.s[0]) && switch_is_mid(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
	{
		//����ҡ�˴�� \../,����2s
		rc_cmd_time++;
	}
	else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_mid(calibrate_RC->rc.s[0]) && switch_is_mid(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
	{
		//����ҡ�˴��'\/',����2s
		rc_cmd_time++;
		rc_action_flag = GIMBAL_FLAG;
	}
	else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_mid(calibrate_RC->rc.s[0]) && switch_is_mid(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
	{
		//����ҡ�˴��./\.,����2s
		rc_cmd_time++;
		rc_action_flag = GYRO_FLAG;
	}
	else
	{
		//�޲���������ң��������ʱ��
		rc_cmd_time = 0;
	}
	
	//��¼У׼ϵͳʱ��
	calibrate_systemTick = xTaskGetTickCount();

	if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
	{
		//�����յ�ң����ָ�ʼ(rc_cmd_systemTick)����20��δ���У׼, ��ǿ����ֹ���У׼
		rc_action_flag = 0;
		return;
	}
	else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
	{
		rc_cali_buzzer_middle_on();
	}
	else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
	{
		rc_cali_buzzer_start_on();
	}

	if (rc_action_flag != 0)
	{
		buzzer_time++;
	}
    
	if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
	{
		buzzer_time = 0;
	}
	
	if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
	{
		cali_buzzer_off();
	}
}

/**
  * @brief          ��flash��ȡУ׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
	uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
	uint16_t offset = 0;
	for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++)
	{
		//��flash�ж�ȡ����
		cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
		
		offset += cali_sensor[i].flash_len * 4;

		//��flash�ж�ȡ�豸���ƺ��Ƿ����У׼�ı�־
		cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
		
		cali_sensor[i].name[0] = flash_read_buf[0];
		cali_sensor[i].name[1] = flash_read_buf[1];
		cali_sensor[i].name[2] = flash_read_buf[2];
		cali_sensor[i].cali_done = flash_read_buf[3];
		
		offset += CALI_SENSOR_HEAD_LEGHT * 4;

		if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
		{
				cali_sensor[i].cali_cmd = 1;
		}
	}
}

/**
  * @brief          ��flashд��У׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
	uint8_t i = 0;
	uint16_t offset = 0;
	
	for (i = 0; i < CALI_LIST_LENGHT; i++)
	{
		//���� �豸У׼ֵ
		memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
		offset += cali_sensor[i].flash_len * 4;

		//���� �豸���� �� �豸�Ƿ����У׼�ı�־
		memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
		offset += CALI_SENSOR_HEAD_LEGHT * 4;
	}
	//����flash����ص�ַ
	cali_flash_erase(FLASH_USER_ADDR, 1);
	//д������
	cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
  * @brief          У׼������main��������
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void calibrate_task(void const *pvParameters)
{
	//��ȡң����ָ��
	calibrate_RC = get_remote_ctrl_point_cali();

	while (1)
	{
		//ʹ��ң������ʼУ׼
		RC_cmd_to_calibrate();
	
		for (uint8_t i = 0; i < CALI_LIST_LENGHT; i++) {
			if (cali_sensor[i].cali_cmd) {
				if (cali_sensor[i].cali_hook != NULL) {
					if (cali_sensor[i].cali_hook(cali_data_addr_buf[i], CALI_FUNC_CMD_ON)) {
						cali_sensor[i].name[0] = cali_name[i][0];	// У׼���
						cali_sensor[i].name[1] = cali_name[i][1];
						cali_sensor[i].name[2] = cali_name[i][2];
						cali_sensor[i].cali_done = CALIED_FLAG; // ����У׼��־
						cali_sensor[i].cali_cmd = 0;
						cali_data_write(); // д��Flash
					}
				}
			}
		}
		
		osDelay(CALIBRATE_CONTROL_TIME);
		
		#if INCLUDE_uxTaskGetStackHighWaterMark
			calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
		#endif
	}
}

/**
  * @brief          "head"�豸У׼
  * @param[in][out] cali:ָ��ָ��head����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
	head_cali_t *local_cali_t = (head_cali_t *)cali;
	if (cmd == CALI_FUNC_CMD_INIT)
	{
		memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));
		return 1;
	}
	local_cali_t->self_id = SELF_ID; // ����ID
	local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10; // IMU�����¶�(����MCU�¶�+10��C)
	//IMU�¶��޷�
	if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
	{
		local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
	}
	local_cali_t->firmware_version = FIRMWARE_VERSION; // �̼��汾
	local_cali_t->latitude = 22.0f; // Ĭ��γ��(22.0f������γ��)	

	return 1;
}

/**
  * @brief          �������豸У׼
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
	imu_cali_t *local_cali_t = (imu_cali_t *)cali;
	if (cmd == CALI_FUNC_CMD_INIT)
	{
		gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
		return 0;
	}
	else if (cmd == CALI_FUNC_CMD_ON)
	{
		static uint16_t count_time = 0;
		gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
		if (count_time > GYRO_CALIBRATE_TIME)
		{
			count_time = 0;
			cali_buzzer_off();
			gyro_cali_enable_control();
			return 1;
		}
		else
		{
			gyro_cali_disable_control(); //disable the remote control to make robot no move
			imu_start_buzzer();
			return 0;
		}
	}

	return 0;
}

/**
  * @brief          ��̨�豸У׼
  * @param[in][out] cali:ָ��ָ����̨����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{
	gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
	if (cmd == CALI_FUNC_CMD_INIT)
	{
		set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
												 local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
												 local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
		return 0;
	}
	else if (cmd == CALI_FUNC_CMD_ON)
	{
		if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
														 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
														 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
		{
			cali_buzzer_off();
			return 1;
		}
		else
		{
			gimbal_start_buzzer();
			return 0;
		}
	}
	
	return 0;
}
