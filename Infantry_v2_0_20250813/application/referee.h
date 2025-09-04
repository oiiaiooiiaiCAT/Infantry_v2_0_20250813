#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"


typedef enum
{
	RED_HERO        = 1,     // �췽Ӣ�ۻ�����
	RED_ENGINEER    = 2,     // �췽���̻�����
	RED_STANDARD_1  = 3,     // �췽1�Ų���������
	RED_STANDARD_2  = 4,     // �췽2�Ų���������
	RED_STANDARD_3  = 5,     // �췽3�Ų���������
	RED_AERIAL      = 6,     // �췽���л�����
	RED_SENTRY      = 7,     // �췽�ڱ�������
	RED_STATE       = 11,    // �췽״̬��Ϣ
	BLUE_HERO       = 101,   // ����Ӣ�ۻ�����
	BLUE_ENGINEER   = 102,   // �������̻�����
	BLUE_STANDARD_1 = 103,   // ����1�Ų���������
	BLUE_STANDARD_2 = 104,   // ����2�Ų���������
	BLUE_STANDARD_3 = 105,   // ����3�Ų���������
	BLUE_AERIAL     = 106,   // �������л�����
	BLUE_SENTRY     = 107,   // �����ڱ�������
	BLUE_STATE      = 111,   // ����״̬��Ϣ
  
} robot_id_e;

typedef enum
{
	PROGRESS_UNSTART        = 0,  // ����δ��ʼ
	PROGRESS_PREPARE        = 1,  // ׼���׶�
	PROGRESS_SELFCHECK      = 2,  // �Լ�׶�
	PROGRESS_5sCOUNTDOWN    = 3,  // 5�뵹��ʱ
	PROGRESS_BATTLE         = 4,  // ��ս�׶�
	PROGRESS_CALCULATING    = 5,  // �������׶�
  
} game_progress_e;

typedef __packed struct //0x0001
{
	uint8_t game_type : 4;      	//��������
	uint8_t game_progress : 4;  	//��ǰ�����׶�
	uint16_t stage_remain_time; 	//��ǰ�׶�ʣ��ʱ�䣬��λ����
	uint64_t SyncTimeStamp;				//UNIX ʱ�䣬����������ȷ���ӵ�����ϵͳ�� NTP ����������Ч
	
} ext_game_state_t;

typedef __packed struct //0x0002
{
	uint8_t winner;		//0��ƽ��  1���췽ʤ��  2������ʤ��
	
} ext_game_result_t;

typedef __packed struct//0x0003
{
	uint16_t red_1_robot_HP;		//�� 1 Ӣ�ۻ�����Ѫ�������û�����δ�ϳ����߱����£���Ѫ��Ϊ 0
	uint16_t red_2_robot_HP;  	//�� 2 ���̻�����Ѫ��
	uint16_t red_3_robot_HP;  	//�� 3 ����������Ѫ��
	uint16_t red_4_robot_HP;  	//�� 4 ����������Ѫ��
	uint16_t red_5_robot_HP;  	//�� 5 ����������Ѫ��
	uint16_t red_7_robot_HP;  	//�� 7 �ڱ�������Ѫ��
	uint16_t red_outpost_HP;  	//�췽ǰ��վѪ��
	uint16_t red_base_HP;     	//�췽����Ѫ��
	uint16_t blue_1_robot_HP; 	//�� 1 Ӣ�ۻ�����Ѫ��
	uint16_t blue_2_robot_HP; 	//�� 2 ���̻�����Ѫ��
	uint16_t blue_3_robot_HP; 	//�� 3 ����������Ѫ��
	uint16_t blue_4_robot_HP; 	//�� 4 ����������Ѫ��
	uint16_t blue_5_robot_HP; 	//�� 5 ����������Ѫ��
	uint16_t blue_7_robot_HP; 	//�� 7 �ڱ�������Ѫ��
	uint16_t blue_outpost_HP; 	//����ǰ��վѪ��
	uint16_t blue_base_HP;    	//��������Ѫ��
	
} ext_game_robot_HP_t;

typedef __packed struct //0x0101
{
	uint32_t event_type;	// �¼�����
	
} ext_event_data_t;

typedef __packed struct //0x0102 ���貹����������
{
	uint8_t supply_projectile_id;     // ��������ID
	uint8_t supply_robot_id;          // ����������ID
	uint8_t supply_projectile_step;   // �������貽��
	uint8_t supply_projectile_num;    // ������������
	
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103 ���貹��Ԥ������
{
	uint8_t reserved;                 // ����λ
	uint8_t supply_robot_id;          // ����������ID
	uint8_t supply_projectile_step;   // �������貽��
	uint8_t supply_projectile_num;    // ������������
	
} ext_supply_projectile_booking_t;

typedef __packed struct	//0x0104 ���о�������
{
	uint8_t level;                    // ����ȼ�
	uint8_t offending_robot_id;       // Υ�������ID
	uint8_t count;                    // Υ�����
	
} ext_referee_warning_t;

typedef __packed struct	// 0x0105: ������Ϣ
{
	uint8_t dart_remaining_time;      // ����ʣ��ʱ��
	uint16_t dart_info;               // ������Ϣ
	
}dart_info_t;


typedef __packed struct //0x0201
{
	uint8_t robot_id;       //�������� ID
	uint8_t robot_level;    //�����˵ȼ�
	uint16_t current_HP;    //�����˵�ǰѪ��
	uint16_t maximum_HP;    //������Ѫ������
	uint16_t shooter_barrel_cooling_value; 				//������ǹ������ÿ����ȴֵ
	uint16_t shooter_barrel_heat_limit;    				//������ǹ����������
	uint16_t chassis_power_limit;          				//�����˵��̹�������**
	uint8_t power_management_gimbal_output : 1;		//��Դ����ģ�����������gimbal �������0 Ϊ�������1 Ϊ 24V ���
	uint8_t power_management_chassis_output : 1;	//chassis �������0 Ϊ�������1 Ϊ 24V ���
	uint8_t power_management_shooter_output : 1;	//shooter �������0 Ϊ�������1 Ϊ 24V ���
	
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
	uint16_t chassis_voltage;  //��Դ����ģ��� chassis �������ѹ����λ��mV��
	uint16_t chassis_current;  //��Դ����ģ��� chassis �������������λ��mA��
	float chassis_power;       //���̹��ʣ���λ��W��
	uint16_t buffer_energy;    //������������λ��J��**
	uint16_t shooter_17mm_barrel_heat;  	//17mm ���������ǹ������
//	uint16_t shooter_17mm_2_barrel_heat;  //�� 2 �� 17mm ���������ǹ������
	uint16_t shooter_42mm_barrel_heat;    //42mm ���������ǹ������
	
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
	float x;      //��������λ�� x ���꣬��λ��m
	float y;      //��������λ�� y ���꣬��λ��m
	float angle;  //�������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ 0 ��

} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
	uint8_t recovery_buff; 			//�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��
	uint8_t cooling_buff;				//������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ��
	uint8_t defence_buff;  			//�����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
	uint8_t vulnerability_buff;	//�����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ-30%�������棩
	uint16_t attack_buff;  			//�����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
	
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
	uint8_t airforce_status;  //���л�����״̬��0 Ϊ������ȴ��1 Ϊ��ȴ��ϣ�2 Ϊ���ڿ���֧Ԯ��
	uint8_t time_remain;  		//��״̬��ʣ��ʱ�䣨��λΪ���룬����ȡ��������ȴʱ��ʣ�� 1.9 ��ʱ����ֵΪ 1������ȴʱ��Ϊ 0����δ���п���֧Ԯ�����ֵΪ 0

} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
	uint8_t armor_id : 4;							//����Ѫԭ��Ϊװ��ģ�鱻���蹥������ײ�������߻����ģ������ʱ���� 4 bit ��ɵ���ֵΪװ��ģ������ģ��� ID ��ţ�
																		//������ԭ���¿�Ѫʱ������ֵΪ 0
	uint8_t HP_deduction_reason : 4;  //Ѫ���仯����
	
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
	uint8_t bullet_type;          //��������
	uint8_t shooter_number;       //������� ID��
	uint8_t launching_frequency;  //�������٣���λ��Hz��
	float initial_speed;          //������ٶȣ���λ��m/s��
	
} ext_shoot_data_t;

typedef __packed struct //0x0208
{
	uint16_t projectile_allowance_17mm;  //17mm ������������
	uint16_t projectile_allowance_42mm;  //42mm ������������
	uint16_t remaining_gold_coin;        //ʣ��������
	
} projectile_allowance_t;

typedef __packed struct //0x0209
{
	uint32_t rfid_status;		// RFID״̬��Ϣ
	
} rfid_status_t;

typedef __packed struct	//0x020A
{
	uint8_t bullet_remaining_num;		// ����ʣ������
	
}	ext_bullet_remaining_t;

typedef __packed struct //0x020B
{
	uint8_t dart_launch_opening_status;   // ���ڷ��俪��״̬
	uint8_t reserved;                     // ����λ
	uint16_t target_change_time;          // Ŀ����ʱ��
	uint16_t latest_launch_cmd_time;      // �����������ʱ��
	
} dart_client_cmd_t;

typedef __packed struct //0x020C
{   
	float hero_x;        //����Ӣ�ۻ�����λ�� x �����꣬��λ��m
	float hero_y;        //����Ӣ�ۻ�����λ�� y �����꣬��λ��m
	float engineer_x;    //�������̻�����λ�� x �����꣬��λ��m
	float engineer_y;    //�������̻�����λ�� y �����꣬��λ��m
	float standard_3_x;  //���� 3 �Ų���������λ�� x �����꣬��λ��m
	float standard_3_y;  //���� 3 �Ų���������λ�� y �����꣬��λ��m
	float standard_4_x;  //���� 4 �Ų���������λ�� x �����꣬��λ��m
	float standard_4_y;  //���� 4 �Ų���������λ�� x �����꣬��λ��m
	float standard_5_x;  //���� 5 �Ų���������λ�� x �����꣬��λ��m
	float standard_5_y;  //���� 5 �Ų���������λ�� y �����꣬��λ��m
	
}	ground_robot_position_t;

typedef __packed struct //0x020D
{   
	uint8_t mark_hero_progress;         // Ӣ�ۻ����˱�ǽ���
	uint8_t mark_engineer_progress;     // ���̻����˱�ǽ���
	uint8_t mark_standard_3_progress;   // 3�Ų��������˱�ǽ���
	uint8_t mark_standard_4_progress;   // 4�Ų��������˱�ǽ���
	uint8_t mark_standard_5_progress;   // 5�Ų��������˱�ǽ���
	uint8_t mark_sentry_progress;       // �ڱ������˱�ǽ���
	
}radar_mark_data_t; 

typedef __packed struct  //0x020E
{
	uint32_t sentry_info;		// �ڱ���Ϣ
	
} sentry_info_t;

typedef __packed struct  //0x020F
{
	uint8_t radar_info;			// �״���Ϣ
	
} radar_info_t;

typedef __packed struct //0x0301
{
	uint16_t data_cmd_id;  		// ��������ID
	uint16_t sender_id;    		// ������ID
	uint16_t receiver_id;  		// ������ID
	uint8_t user_data[113];		// �û�����
	
} ext_student_interactive_data_t;

typedef __packed struct __CustomControllerData  //0x0302 �Զ������������
{
	uint8_t data[30];		// ����������
	
} custom_controller_data_t;

typedef __packed struct //0x0303
{ 
	float target_position_x;		// Ŀ��λ��x����
	float target_position_y;		// Ŀ��λ��y����
	uint8_t cmd_keyboard;   		// ��������
	uint8_t target_robot_id;		// Ŀ�������ID
	uint8_t cmd_source;     		// ������Դ
	
}	map_command_t; 

typedef __packed struct	//0x0304
{
	uint16_t mouse_x;           	// ���X����
	uint16_t mouse_y;           	// ���Y����
	uint16_t mouse_z;           	// ���Z���꣨���֣�
	uint8_t left_button_down;   	// �������״̬
	uint8_t right_button_down;  	// �Ҽ�����״̬
	uint16_t keyboard_value;    	// ����ֵ
	uint16_t reserved;          	// ����λ
	
} ext_robot_command_t;

typedef __packed struct //0x0305
{ 
	uint16_t target_robot_id;     // Ŀ�������ID
	float target_position_x;      // Ŀ��λ��x����
	float target_position_y;      // Ŀ��λ��y����
	
}	map_robot_data_t; 

typedef __packed struct //0x0307
{ 
	uint8_t intention;            // ��ͼ
	uint16_t start_position_x;    // ��ʼλ��x����
	uint16_t start_position_y;    // ��ʼλ��y����
	int8_t delta_x[49];           // x������������
	int8_t delta_y[49];           // y������������
	uint16_t sender_id;           // ������ID
	
}	map_data_t; 

typedef __packed struct	//0x0308
{ 
	uint16_t sender_id;        		// ������ID
	uint16_t receiver_id;      		// ������ID
	uint8_t user_data[30];     		// �û�����
	
} custom_info_t; 

typedef __packed struct	//0x0309 �Զ�������
{
    float data1;               // ����1
    float data2;               // ����2
    float data3;               // ����3
    uint8_t data4;             // ����4
	
} custom_data_t;

typedef __packed struct	//0x030A ����������
{
	uint8_t data[64];
	
} ext_up_stream_data_t;

typedef __packed struct	//0x030B ����������
{
	uint8_t data[32];
	
} ext_download_stream_data_t;

extern void init_referee_data(void);
extern void referee_data_solve(uint8_t *frame);
extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern uint8_t get_robot_id(void);
extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat_limit, uint16_t *heat);


#endif
