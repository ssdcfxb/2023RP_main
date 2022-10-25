/**
  ******************************************************************************
  * @file           : judge.c
  * @brief          : 
  * @update         : finish 2022-2-13 20:06:34
  ******************************************************************************
  */

#include "judge.h"
#include "string.h"

judge_info_t judge_info;
judge_t judge = 
{
	.info = &judge_info,
};


void judge_update(uint16_t id, uint8_t *rxBuf)
{
	switch(id)
	{
		case ID_rfid_status:
			memcpy(&judge.info->rfid_status, rxBuf, LEN_rfid_status);
			break;
		case ID_game_state:
			memcpy(&judge.info->game_status, rxBuf, LEN_game_state);
			break;
		case ID_power_heat_data:
			memcpy(&judge.info->power_heat_data, rxBuf, LEN_power_heat_data);
			up_send_power_heat();
			break;
		case ID_game_robot_state:	
			memcpy(&judge.info->game_robot_status, rxBuf, LEN_game_robot_state);
			up_send_game_robot_state();
			break;
//		case ID_shoot_data:
//			memcpy(&judge.info->shoot_data, rxBuf, LEN_shoot_data);
//			up_send_shoot();
//			break;
//		case ID_game_robot_pos:
//			memcpy(&judge.info->game_robot_pos, rxBuf, LEN_game_robot_pos);
//			up_send_robot_pos();
//			break;
		case ID_robot_hurt:
			memcpy(&judge.info->ext_robot_hurt, rxBuf, LEN_robot_hurt);
			up_send_robot_hurt();
			break;
		default:
			break;
	}
}
