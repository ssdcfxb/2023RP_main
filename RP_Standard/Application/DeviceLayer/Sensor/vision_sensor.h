#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H

#include "rp_config.h"
#include "rp_math.h"
#include "string.h"

#include "crc.h"
#include "rc_sensor.h"

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


typedef struct
{
	uint8_t  mode;
	uint8_t  cmd_mode;
	uint8_t  color;
	
	float    measure_pitch_angle;
	float    measure_yaw_angle;
	uint8_t  measure_shoot_speed;
	float    target_pitch_angle;
	float    target_yaw_angle;
	uint8_t  is_find_target;
	uint8_t  is_find_defund;
	uint8_t  is_hit_enable;
	
	vision_tx_info_t  *tx_info;
	vision_rx_info_t  *rx_info;
	uint8_t           rx_flag;
	int16_t		        offline_cnt;
	int16_t		        offline_max_cnt;
}vision_info_t;

typedef struct vision_sensor_struct {
	vision_info_t	    *info;
	drv_uart_t		    *driver;
	void				     (*init)(struct vision_sensor_struct *self);
	void				     (*update)(struct vision_sensor_struct *self, uint8_t *rxBuf);
	void				     (*check)(struct vision_sensor_struct *self);	
	void				     (*heart_beat)(struct vision_sensor_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			  errno;
	dev_id_t			    id;
} vision_sensor_t;


extern vision_tx_info_t vision_tx_info;
extern vision_rx_info_t vision_rx_info;
extern vision_sensor_t vision_sensor;

extern bool vision_send_data(void);

#endif
