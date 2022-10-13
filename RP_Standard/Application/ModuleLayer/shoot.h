#ifndef __SHOOT_H
#define __SHOOT_H

#include "rp_config.h"

#include "gimbal.h"
#include "rc_sensor.h"
#include "motor_3508.h"
#include "motor_2006.h"

//// Ħ����ģʽö��
//typedef enum
//{
//	G_Y_gyro,      // ������
//	G_Y_machine,   // ��е
//	G_Y_keep,      // ����
//} shoot_mode_e;

//// ����ģʽö��
//typedef enum
//{
//	G_P_gyro,      // ������
//	G_P_machine,   // ��е
//	G_P_keep,      // ����
//} dial_mode_e;

// �������ģʽö��
typedef enum
{
	Shoot_Keep,     // ����
	Shoot_Single,   // ����
	Shoot_Reset,    // ��λ
} launcher_mode_e;

typedef struct 
{
	motor_3508_t  *shoot_left;
	motor_3508_t  *shoot_right;
	motor_2006_t  *dial_motor;
	rc_sensor_t		*rc_sensor;
} shoot_dev_t;

typedef struct
{
	remote_mode_t		 remote_mode;
	launcher_mode_e  launcher_mode;
//	shoot_mode_e     shoot_mode;
//	dial_mode_e      dial_mode;
	int16_t  measure_left_speed;
	int16_t  measure_right_speed;
	int16_t  measure_dial_speed;
	int16_t  measure_dial_angle;
	
	int16_t  target_left_speed;
	int16_t  target_right_speed;
	int16_t  target_dial_speed;
	int16_t  target_dial_angle;
	
} shoot_info_t;

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
} shoot_conf_t;

typedef struct 
{
	shoot_dev_t    *dev;
	shoot_info_t   *info;
	shoot_conf_t   *conf;
	void			(*init)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
} launcher_t;


//void Shoot_GetInfo(void);
//void Shoot_Stop(void);
//void Shoot_RcCtrl(void);
//void Shoot_PidCtrl(void);

#endif
