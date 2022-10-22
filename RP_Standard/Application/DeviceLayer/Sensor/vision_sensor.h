#ifndef __VISION_SENSOR_H
#define __VISION_SENSOR_H

#include "rp_config.h"
#include "rp_math.h"

#include "gimbal.h"
#include "rc_sensor.h"


typedef struct
{
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
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


#endif
