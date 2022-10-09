#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rp_math.h"

#include "motor_3508.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "gimbal.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	CHASSIS_MODE_NORMAL 		= 0, // 正常模式
	CHASSIS_MODE_BUFF   		= 1, // 打符模式
	CHASSIS_MODE_RELOAD_BULLET	= 2, // 底盘低速补弹模式
	CHASSIS_MODE_SZUPUP			= 3, // SZU爬坡模式
	CHASSIS_MODE_GYRO       = 4, // 小陀螺模式
} chassis_mode_t;

typedef struct {
	motor_3508_t	*chas_motor[CHAS_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		  remote_mode;
	chassis_mode_t		local_mode;
	int16_t target_front_speed;  //目标前进速度
	int16_t target_right_speed;  //目标左移速度
	int16_t target_cycle_speed;  //目标旋转速度
	int16_t measure_front_speed; //当前前进速度
	int16_t measure_right_speed; //当前左移速度
	int16_t measure_cycle_speed; //当前旋转速度
}chassis_info_t;

typedef struct {
	float  limit_speed; // 电机最大转速
	float  machine_round_kp; // 机械模式底盘跟随角速度系数
	float  gyro_round_speed; // 小陀螺最大转速
} chassis_conf_t;

typedef struct chassis{
	chassis_dev_t 	*dev;
	chassis_info_t	*info;
	chassis_conf_t  *conf;
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
/* 信息层 --------------------------------------------------------------------*/

#endif
