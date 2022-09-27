#include "pid.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//#define SPEED_KP 1.0f
//#define SPEED_KI 0.01f
//#define SPEED_KD 0.15f
		
#define SPEED_KP 0.005f
#define SPEED_KI 0.0f
#define SPEED_KD 0.0f

#define ANGLE_KP 1.3f
#define ANGLE_KI 0.00005f
#define ANGLE_KD 8.0f
		
float PID_Speed[3] = {SPEED_KP, SPEED_KI, SPEED_KD};
float PID_Angle[3] = {ANGLE_KP, ANGLE_KI, ANGLE_KD};

void PID_Init(PID_Type_Def *pid, float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
		
}
		
float PID_Inc_Calc(PID_Type_Def *pid, float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    pid->error[0] = set - fdb;
//		pid->Dbuf[0] = pid->error[0] - pid->error[1]; //0 误差增量 1 误差增量的增量 2 上一次误差增量
//		pid->Dbuf[1] = pid->Dbuf[0] - pid->Dbuf[2];
		pid->Pout = pid->Kp * (1.0f + pid->Ki + pid->Kd) * pid->error[0];
		pid->Iout = pid->Kp * (1.0f + pid->Kd + pid->Kd) * pid->error[1];
		pid->Dout = pid->Kd * pid->error[2];
//		LimitMax(pid->Iout, pid->max_iout);
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
    
    return pid->out;
}

float PID_Plc_Calc(PID_Type_Def *pid, float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    pid->error[0] = set - fdb;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
    
    return pid->out;
}

