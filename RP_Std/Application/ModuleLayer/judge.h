#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"

#include "judge_protocol.h"

typedef struct 
{
	ext_rfid_status_t rfid_status;
	ext_game_status_t game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t power_heat_data;
	ext_shoot_data_t shoot_data;
	ext_robot_hurt_t ext_robot_hurt;
}judge_info_t;

typedef struct 
{
	judge_info_t *info;
}judge_t;

extern judge_t judge;

void Send_Can(void);
void judge_update(uint16_t id, uint8_t *rxBuf);

#endif
