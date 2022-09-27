#ifndef __PID_H
#define __PID_H
#include "main.h"


#define MAX_OUT 2000.0f
#define MAX_I_OUT 1000.0f

typedef struct __PID_Type_Def
{
    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
		
} PID_Type_Def;


extern float PID_Speed[3];
extern float PID_Angle[3];


float PID_Inc_Calc(PID_Type_Def *pid, float fdb, float set);
float PID_Plc_Calc(PID_Type_Def *pid, float fdb, float set);

extern void PID_Init(PID_Type_Def *pid, float PID[3], float max_out, float max_iout);

#endif
