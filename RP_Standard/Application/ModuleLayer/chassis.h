#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "motor_3508.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	CHASSIS_MODE_NORMAL 		= 0, // ����ģʽ
	CHASSIS_MODE_BUFF   		= 1, // ���ģʽ
	CHASSIS_MODE_RELOAD_BULLET	= 2, // ���̵��ٲ���ģʽ
	CHASSIS_MODE_SZUPUP			= 3, // SZU����ģʽ
} chassis_mode_t;

typedef struct {
	pid_type_t	speed;
	pid_type_t	angle;
	float		speed_out;
	float   angle_out;
} chassis_motor_pid_t;

typedef struct {
	pid_type_t	angle;
	float 	angle_out;
} chassis_z_pid_t;

typedef struct {
	chassis_motor_pid_t		(*motor)[CHAS_MOTOR_CNT];
} chassis_ctrl_t;

typedef struct {
	chassis_motor_t	*chas_motor[CHAS_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		  remote_mode;
	chassis_mode_t		local_mode;
}chassis_info_t;

typedef struct chassis{
	chassis_ctrl_t	*controller;
	chassis_dev_t 	*dev;
	chassis_info_t	*info;
	bool			test_open;
	void			(*init)(void);
	void			(*update)(void);
	void			(*test)(void);
	void			(*ctrl)(void);
	void			(*output)(void);
	void			(*self_protect)(void);
}chassis_t;

extern chassis_t chassis;

/* Exported functions --------------------------------------------------------*/
/* ��Ϣ�� --------------------------------------------------------------------*/

#endif
