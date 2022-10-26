#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"
#include "imu_sensor.h"
#include "vision_sensor.h"
#include "motor_2006.h"
#include "motor_3508.h"
#include "motor_6020.h"
#include "judge.h"


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct {
	rc_sensor_t 		*rc_sen;
	imu_sensor_t		*imu_sen;
//	motor_3508_t		*chas_mtr[CHAS_MOTOR_CNT];
//	motor_3508_t		*fric_mtr[FRIC_MOTOR_CNT];
//	motor_6020_t    *yaw_mtr;
//	motor_6020_t    *pitch_mtr;
//	motor_2006_t    *dial_mtr;
	vision_sensor_t *vis_sen;
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEV_Init(void);

#endif
