#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rp_config.h"

#include "motor_6020.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

// 云台yaw轴模式枚举
typedef enum
{
	G_Y_follow,    // 跟随底盘
	G_Y_gyro,      // 陀螺仪
	G_Y_machine,   // 机械
	G_Y_keep,      // 保持
} gimbal_yaw_mode_e;

// 云台pitch轴模式枚举
typedef enum
{
	G_P_follow,    // 跟随底盘
	G_P_gyro,      // 陀螺仪
	G_P_machine,   // 机械
	G_P_keep,      // 保持
} gimbal_pitch_mode_e;

typedef struct 
{
	motor_6020_t    *yaw_motor;
	motor_6020_t    *pitch_motor;
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} gimbal_dev_t;

typedef struct
{
	remote_mode_t		     remote_mode;
	gimbal_yaw_mode_e    yaw_mode;
	gimbal_pitch_mode_e  pitch_mode;
	float    measure_yaw_imu_speed;
	float    measure_yaw_imu_angle;
	int16_t  measure_yaw_motor_speed;
	int16_t  measure_yaw_motor_angle;
	float    measure_pitch_imu_speed;
	float    measure_pitch_imu_angle;
	int16_t  measure_pitch_motor_speed;
	int16_t  measure_pitch_motor_angle;
	float    target_yaw_imu_angle;
	int16_t  target_yaw_motor_angle;
	float    target_pitch_imu_angle;
	float    target_pitch_motor_angle;
	float    target_pitch_motor_deltaangle;
} gimbal_info_t;

typedef struct 
{
	gimbal_dev_t    *dev;
	gimbal_info_t   *info;
	void			(*init)(void);
	void			(*update)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
} gimbal_t;

extern gimbal_t gimbal;

#endif
