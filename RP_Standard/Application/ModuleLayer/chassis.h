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
	CHASSIS_MODE_NORMAL 		= 0, // ����ģʽ
	CHASSIS_MODE_BUFF   		= 1, // ���ģʽ
	CHASSIS_MODE_RELOAD_BULLET	= 2, // ���̵��ٲ���ģʽ
	CHASSIS_MODE_SZUPUP			= 3, // SZU����ģʽ
	CHASSIS_MODE_GYRO       = 4, // С����ģʽ
} chassis_mode_t;

typedef struct {
	motor_3508_t	*chas_motor[CHAS_MOTOR_CNT];
	imu_sensor_t	*imu_sensor;
	rc_sensor_t		*rc_sensor;
} chassis_dev_t;

typedef struct {
	remote_mode_t		  remote_mode;
	chassis_mode_t		local_mode;
	int16_t target_front_speed;  //Ŀ��ǰ���ٶ�
	int16_t target_right_speed;  //Ŀ�������ٶ�
	int16_t target_cycle_speed;  //Ŀ����ת�ٶ�
	int16_t measure_front_speed; //��ǰǰ���ٶ�
	int16_t measure_right_speed; //��ǰ�����ٶ�
	int16_t measure_cycle_speed; //��ǰ��ת�ٶ�
}chassis_info_t;

typedef struct {
	float  limit_speed; // ������ת��
	float  machine_round_kp; // ��еģʽ���̸�����ٶ�ϵ��
	float  gyro_round_speed; // С�������ת��
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
/* ��Ϣ�� --------------------------------------------------------------------*/

#endif
