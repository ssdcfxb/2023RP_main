#ifndef __PID_H
#define __PID_H
#include "main.h"


//Plc-Inc
#define SPEED_KP 15.0f
#define SPEED_KI 0.5f
#define SPEED_KD 0.5f

#define ANGLE_KP 60.0f
#define ANGLE_KI 0.0f
#define ANGLE_KD 1.0f

//Plc-Plc
//#define SPEED_KP 10.0f
//#define SPEED_KI 0.33f
//#define SPEED_KD 0.0f

//#define ANGLE_KP 25.0f
//#define ANGLE_KI 0.0f
//#define ANGLE_KD 5.0f

#define SP_MAX_OUT 10000.0f
#define SP_MAX_INTEGRAL 3000.0f  //Plc
#define SP_MAX_I_OUT 3000.0f     //Inc

#define AG_MAX_OUT 10000.0f
#define AG_MAX_INTEGRAL 3000.0f  //Plc

typedef struct __pid_type_t
{
    float Kp;
    float Ki;
    float Kd;
	  float	integral;

	  float max_integral;
    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
	
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
		
} pid_type_t;

typedef struct __pid_t{
	pid_type_t	speed;
	pid_type_t	angle;
	float		speed_out;
	float   angle_out;
} pid_t;

float PID_Inc_Calc(pid_type_t *pid, float fdb, float set);
float PID_Plc_Calc(pid_type_t *pid, float fdb, float set);

extern void PID_Init(pid_type_t *pid);

#endif
