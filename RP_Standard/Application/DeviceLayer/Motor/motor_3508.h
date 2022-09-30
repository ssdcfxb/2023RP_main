#ifndef __MOTOR_3508_H
#define __MOTOR_3508_H


#include "rp_config.h"
#include "can_protocol.h"
#include "pid.h"

#define M2006_ECD_TO_ANGLE 0.001220703125
#define M3508_ECD_TO_ANGLE 0.00231319355647
#define GM6020_ECD_TO_ANGLE 0.0439453125
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

typedef struct __chassis_motor_info_t
{
	  volatile uint16_t ecd;
    volatile int16_t  speed_rpm;
    volatile int16_t  given_current;
	  volatile uint8_t  temperature;
    volatile int16_t  last_ecd;
    volatile int16_t  delta_ecd;
    volatile int32_t  total_ecd;
		volatile float    angle;
	  volatile uint8_t  offline_cnt;
	  const    uint8_t	offline_max_cnt;
} chassis_motor_info_t;

typedef struct __chassis_motor_t
{
	  chassis_motor_info_t   *info;
	  drv_can_t              *driver;
	  void					 (*init)(struct __chassis_motor_t *motor);
		void           (*update)(struct __chassis_motor_t *motor, uint8_t* data);
	  void           (*check)(struct __chassis_motor_t *motor);	
	  void					 (*heart_beat)(struct __chassis_motor_t *motor);
	  volatile dev_work_state_t   work_state;
	  volatile dev_errno_t errno;
	  const    dev_id_t		      	id;
	

		PID_Type_Def hpid_angle;
		PID_Type_Def hpid_speed;
	
		float Speed_out; //��������������̨���
		float Angle_out;
	
} chassis_motor_t;


extern chassis_motor_t chassis_motor[CHAS_MOTOR_CNT];

#endif
