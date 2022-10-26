#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"

typedef struct 
{
	float bullet_speed;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t chassis_power_buffer;
	uint16_t chassis_current;
	uint16_t chassis_volt;
	float chassis_power;
}judge_info_t;

typedef struct 
{
	judge_info_t *info;
}judge_t;

extern judge_t judge;

void Send_Can(void);
void judge_update(uint16_t id, uint8_t *rxBuf);

#endif
