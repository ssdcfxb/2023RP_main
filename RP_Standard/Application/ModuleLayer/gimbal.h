#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rp_config.h"
#include "rp_math.h"

#include "motor_6020.h"
#include "imu_sensor.h"
#include "rc_sensor.h"

// ��̨yaw��ģʽö��
typedef enum
{
	G_Y_gyro,      // ������
	G_Y_machine,   // ��е
	G_Y_keep,      // ����
} gimbal_yaw_mode_e;

// ��̨pitch��ģʽö��
typedef enum
{
	G_P_gyro,      // ������
	G_P_machine,   // ��е
	G_P_keep,      // ����
} gimbal_pitch_mode_e;

// ��̨ģʽö��
typedef enum
{
	gim_gyro,      // ������
	gim_machine,   // ��е
	gim_keep,      // ����
} gimbal_mode_e;

typedef struct 
{
	motor_6020_t  *yaw_motor;
	motor_6020_t  *pitch_motor;
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} gimbal_dev_t;

typedef struct
{
	remote_mode_t		     remote_mode;
	gimbal_mode_e        gimbal_mode;
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
	
	int16_t  yaw_motor_angle_err; // ��λǰ��е�Ƕ����
	float    yaw_rad_err; // �������
} gimbal_info_t;

typedef struct
{
	int16_t  MID_VALUE;
	int16_t  restart_yaw_motor_angle;
	float    restart_yaw_imu_angle;
	int16_t  restart_pitch_motor_angle;
	float    restart_pitch_imu_angle;
	
	int16_t  rc_pitch_motor_offset;
	float    rc_yaw_imu_offset;
	float    rc_pitch_imu_offset;
	
	float    max_pitch_imu_angle; // ��
	float    min_pitch_imu_angle; // ��
	int16_t  max_pitch_motor_angle; // ��
	int16_t  min_pitch_motor_angle; // ��
} gimbal_conf_t;

typedef struct 
{
	gimbal_dev_t    *dev;
	gimbal_info_t   *info;
	gimbal_conf_t   *conf;
	void			(*init)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
} gimbal_t;

extern gimbal_t gimbal;

#endif
