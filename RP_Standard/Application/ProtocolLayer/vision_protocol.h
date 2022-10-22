#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

#include "rp_config.h"
#include "string.h"

#include "crc.h"
#include "vision_sensor.h"

typedef __packed struct 
{
	uint8_t  SOF;
	uint8_t  datau8_1; // mode
	uint8_t  CRC8;
	float    dataf_1;  // pitch_angle
	float    dataf_2;  // yaw_angle
	uint8_t  datau8_2; // shoot_speed
 	uint8_t  datau8_3; // my_color
	uint16_t CRC16;
}vision_tx_info_t;


typedef __packed struct 
{
	uint8_t  SOF;
	uint8_t  datau8_1; // mode
	uint8_t  CRC8;
	float    dataf_1;  // pitch_angle
	float    dataf_2;  // yaw_angle
	uint8_t  datau8_2; // is_find_target
	uint8_t  datau8_3; // is_find_defund
	uint8_t  datau8_4; // is_hit_enable
	uint16_t CRC16;
}vision_rx_info_t;

extern vision_tx_info_t vision_tx_info;
extern vision_rx_info_t vision_rx_info;

bool vision_send_data(void);
bool vision_recieve_data(uint8_t *rxBuf);
void USART1_rxDataHandler(uint8_t *rxBuf);
	
#endif
