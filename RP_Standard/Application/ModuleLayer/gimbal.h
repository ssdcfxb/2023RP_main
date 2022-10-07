#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rp_config.h"

#include "motor_6020.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

// ÔÆÌ¨yawÖáÄ£Ê½Ã¶¾Ù
typedef enum
{
	G_Y_follow,    // ¸úËæµ×ÅÌ
	G_Y_gyro,      // ÍÓÂÝÒÇ
	G_Y_machine,   // »úÐµ
	G_Y_keep,      // ±£³Ö
} gimbal_yaw_mode_e;

// ÔÆÌ¨pitchÖáÄ£Ê½Ã¶¾Ù
typedef enum
{
//	G_P_follow,    // ¸úËæµ×ÅÌ
	G_P_gyro,      // ÍÓÂÝÒÇ
	G_P_machine,   // »úÐµ
	G_P_keep,      // ±£³Ö
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
	float    measure_roll_imu_speed;
	float    measure_roll_imu_angle;
	
	float    yaw_real_rate;
	float    pitch_real_rate;
	
	float    target_yaw_imu_angle;
	float    target_yaw_imu_deltaangle;
	int16_t  target_yaw_motor_angle;
	int16_t  target_yaw_motor_deltaangle;
	float    target_pitch_imu_angle;
	float    target_pitch_imu_deltaangle;
	int16_t  target_pitch_motor_angle;
	int16_t  target_pitch_motor_deltaangle;
} gimbal_info_t;

typedef struct
{
	int16_t  restart_yaw_motor_angle;
	float    restart_yaw_imu_angle;
	int16_t  restart_pitch_motor_angle;
	float    restart_pitch_imu_angle;
	
	int16_t  rc_pitch_motor_offset;
	float    rc_yaw_imu_offset;
	float    rc_pitch_imu_offset;
	
	float    max_pitch_imu_angle; // ¸©
	float    min_pitch_imu_angle; // Ñö
	int16_t  max_pitch_motor_angle; // ¸©
	int16_t  min_pitch_motor_angle; // Ñö
} gimbal_conf_t;

typedef struct 
{
	gimbal_dev_t    *dev;
	gimbal_info_t   *info;
	gimbal_conf_t   *conf;
	void			(*init)(void);
	void			(*update)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
} gimbal_t;

extern gimbal_t gimbal;

#endif
