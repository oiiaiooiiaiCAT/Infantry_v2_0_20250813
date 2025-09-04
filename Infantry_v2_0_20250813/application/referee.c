#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"


frame_header_struct_t referee_receive_header;	//���ݽ���֡ͷ
frame_header_struct_t referee_send_header;		//���ݷ���֡ͷ
game_progress_e game_progress;								//��������
ext_game_state_t game_state;									//0x0001	����״̬
ext_game_result_t game_result;								//0x0002	�������
ext_game_robot_HP_t game_robot_HP_t;					//0x0003	����ʱ�����ˡ�ǰ��վ������Ѫ��
ext_event_data_t field_event;									//0x0101	���������¼�
ext_supply_projectile_action_t supply_projectile_action_t;		//0x0102	���貹����������
ext_supply_projectile_booking_t supply_projectile_booking_t;	//0x0103	���貹��Ԥ������
ext_referee_warning_t referee_warning_t;			//0x0104	���о�������
dart_info_t dart_info;												//0x0105	������Ϣ
ext_game_robot_state_t robot_state;						//0x0201	������״̬����
ext_power_heat_data_t power_heat_data;				//0x0202	������ǹ����������
ext_game_robot_pos_t game_robot_pos;					//0x0203	������λ������
ext_buff_musk_t buff_musk_t;									//0x0204	��������������
aerial_robot_energy_t aerial_robot_energy;		//0x0205	���л�������Ϣ
ext_robot_hurt_t robot_hurt_t;								//0x0206	�����˿�Ѫ��Ϣ
ext_shoot_data_t shoot_data_t;								//0x0207	��������
projectile_allowance_t projectile_allowance;	//0x0208	�ɷ��䵯�������뾭������
rfid_status_t rfid_status;										//0x0209	RFID״̬��Ϣ
ext_student_interactive_data_t student_interactive_data;		//0x0301	ѧ����������
custom_controller_data_t customcontrollerdata;							//0x0302	�Զ������������
map_command_t map_command;										//0x0303	��ͼָ��
ext_robot_command_t robot_command_t;					//0x0304	������ָ��
map_robot_data_t map_robot_data;							//0x0305	��ͼ�ϻ���������
map_data_t map_data;													//0x0307	��ͼ����
custom_info_t custom_info;										//0x0308	�û���Ϣ


/**
	*	@brief				��ʼ������ϵͳ����
	*/
void init_referee_data(void)
{
	memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
	memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

	memset(&game_progress,0,sizeof(game_progress_e));
	memset(&game_state, 0, sizeof(ext_game_state_t));					//0001
	memset(&game_result, 0, sizeof(ext_game_result_t));				//0002
	memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));	//0003

	memset(&field_event, 0, sizeof(ext_event_data_t));				//0101
	memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));			//0102
	memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));		//0103
	memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));			//0104
	memset(& dart_info,0,sizeof(dart_info_t));												//0105

	memset(&robot_state, 0, sizeof(ext_game_robot_state_t));					//0201
	memset(&power_heat_data, 0, sizeof(ext_power_heat_data_t));				//0202
	memset(&game_robot_pos, 0, sizeof(ext_game_robot_pos_t));					//0203
	memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));									//0204
	memset(&aerial_robot_energy, 0, sizeof(aerial_robot_energy_t));		//0205
	memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));								//0206
	memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));								//0207
	memset(& projectile_allowance,0,sizeof(projectile_allowance_t));	//0208
	memset(& rfid_status,0,sizeof(rfid_status_t));										//0209
	
	memset(&student_interactive_data, 0, sizeof(ext_student_interactive_data_t));			//0301
	memset(&customcontrollerdata, 0, sizeof(custom_controller_data_t));								//0302
	memset(& map_command,0,sizeof(map_command_t));						//0303
	memset(& robot_command_t,0,sizeof(ext_robot_command_t));	//0304
	memset(& map_robot_data,0,sizeof(map_robot_data_t));			//0305
	memset(& map_data,0,sizeof(map_data_t));									//0307
	memset(& custom_info,0,sizeof(custom_info_t));						//0308
}

/**
	*	@brief				����ϵͳ���ݴ���
	*	@param[in]		frame ָ��Ӳ���ϵͳ���յ���ԭʼ����֡��ָ��
	*/
void referee_data_solve(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;

	memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

	index += sizeof(frame_header_struct_t);

	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	switch (cmd_id)
	{
		case GAME_STATE_CMD_ID://0001
		{
			memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
		}
		break;
		case GAME_RESULT_CMD_ID://0002
		{
			memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
		}
		break;
		case GAME_ROBOT_HP_CMD_ID://0003
		{
			memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
		}
		break;
		case FIELD_EVENTS_CMD_ID://0101
		{
			memcpy(&field_event, frame + index, sizeof(ext_event_data_t));
		}
		break;
		case SUPPLY_PROJECTILE_ACTION_CMD_ID://0102
		{
			memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
		}
		break;
		case SUPPLY_PROJECTILE_BOOKING_CMD_ID://0103
		{
			memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
		}
		break;
		case REFEREE_WARNING_CMD_ID://0104
		{
			memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
		}
		break;
		 case   DART_INFO_CMD_ID://0105
		{
			memcpy(&dart_info, frame + index, sizeof(dart_info_t));
		}
		break;
		case ROBOT_STATE_CMD_ID://0201
		{
			memcpy(&robot_state, frame + index, sizeof(ext_game_robot_state_t));
		}
		break;
		case POWER_HEAT_DATA_CMD_ID://0202
		{
			memcpy(&power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
		}
		break;
		case ROBOT_POS_CMD_ID://0203
		{
			memcpy(&game_robot_pos, frame + index, sizeof(ext_game_robot_pos_t));
		}
		break;
		case BUFF_MUSK_CMD_ID://0204
		{
			memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
		}
		break;
		case AERIAL_ROBOT_ENERGY_CMD_ID://0205
		{
			memcpy(&aerial_robot_energy, frame + index, sizeof(aerial_robot_energy_t));
		}
		break;
		case ROBOT_HURT_CMD_ID://0206
		{
			memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
		}
		break;
		case SHOOT_DATA_CMD_ID://0207
		{
			memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
		}
		break;
		case PROJECTILE_ALLOWANCE_CMD_ID://0208
		{
			memcpy(&projectile_allowance,frame+index,sizeof(projectile_allowance_t));
		}
		break;
		case RFID_STATUS_CMD_ID://0209
		{
			memcpy(&rfid_status,frame+index,sizeof(rfid_status_t));
		}
		break;
		case STUDENT_INTERACTIVE_DATA_CMD_ID://0301
		{
			memcpy(&student_interactive_data, frame + index, sizeof(ext_student_interactive_data_t));
		}
		break;
		case CUSTOM_CONTROLLER_CMD_ID://0302
		{
			memcpy(&customcontrollerdata, frame + index, sizeof(custom_controller_data_t));
		}
		break;
	 
		case MAP_COMMAND_CMD_ID://0303
		{
			memcpy(&map_command,frame+index,sizeof(map_command_t));
		}
		break;
		case  ROBOT_COMMAND_CMD_ID://0304
		{
			memcpy(&robot_command_t,frame+index,sizeof(ext_robot_command_t));
		}
		break;
		case MAP_ROBOT_DATA_CMD_ID://0305
		{
			memcpy(&map_robot_data,frame+index,sizeof(map_robot_data_t));
		}
		break;
		case MAP_DATA_CMD_ID://0307
		{
			memcpy(&map_data,frame+index,sizeof(map_data_t));
		}
		break;
		case CUSTOM_INFO_CMD_ID://0308
		{
			memcpy(&custom_info,frame+index,sizeof(custom_info_t));
		}
		break;
		
		default:
		{
			break;
		}
	}
}

/**
	*	@brief				��ȡ���̵�ǰ�����뻺������
	*	@param[out]		power		���̵�ǰ�����ù���
	*	@param[out]		buffer	����ʣ�໺������
	*/
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
	*power = robot_state.chassis_power_limit;
	*buffer = power_heat_data.buffer_energy;
}

/**
	*	@brief				��ȡ������ID
	*/
uint8_t get_robot_id(void)
{
	return robot_state.robot_id;
}

/**
	*	@brief				��ȡ���̵�ǰ�����뻺������
	*	@param[out]		heat_limit	������ǹ����������
	*	@param[out]		heat				17mm ���������ǹ������
	*/
void get_shoot_heat0_limit_and_heat0(uint16_t *heat_limit, uint16_t *heat)
{
	*heat_limit = robot_state.shooter_barrel_heat_limit;
	*heat = power_heat_data.shooter_17mm_barrel_heat;
}
