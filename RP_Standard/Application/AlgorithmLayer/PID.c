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
		pid->integral = 0;
}
		
float PID_Inc_Calc(PID_Type_Def *pid, float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
		float PIout = 0.0f;
		float PIDout = 0.0f;
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
		
		//数据更新
		
    //pid->error[0] = set - fdb;
		arm_sub_f32(&set, &fdb, &pid->error[0], 1);
		
		//pid->Dbuf[0] = pid->error[0] - pid->error[1]; //0 误差增量 1 误差增量的增量 2 上一次误差增量
		arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[0], 1);
		
		//pid->Dbuf[1] = pid->Dbuf[0] - pid->Dbuf[2];
		arm_sub_f32(&pid->Dbuf[0], &pid->Dbuf[2], &pid->Dbuf[1], 1);
		pid->Dbuf[2] = pid->Dbuf[0];
		
		//分别计算PID
		//output = kp * DError + ki * Error + kd * DDError
		
		//pid->Pout = pid->Kp * pid->Dbuf[0];
		arm_mult_f32(&pid->Kp, &pid->Dbuf[0], &pid->Pout, 1);
		
		//pid->Iout = pid->Ki * pid->error[0];
		arm_mult_f32(&pid->Ki, &pid->error[0], &pid->Iout, 1);
		LimitMax(pid->Iout, pid->max_iout);
		
		//pid->Dout = pid->Kd * pid->Dbuf[1];
		arm_mult_f32(&pid->Kd, &pid->Dbuf[1], &pid->Dout, 1);
		
		//pid->out += pid->Pout + pid->Iout + pid->Dout;
		arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
		arm_add_f32(&PIout, &pid->Dout, &PIDout, 1);
		arm_add_f32(&PIDout, &pid->out, &pid->out, 1);
		
		LimitMax(pid->out, pid->max_out);
    
    return pid->out;
}

float PID_Plc_Calc(PID_Type_Def *pid, float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

		float PIout = 0.0f;
		
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
		
		//数据更新
		
    //pid->error[0] = set - fdb;
		arm_sub_f32(&set, &fdb, &pid->error[0], 1);
		//pid->integral += pid->error[0];
		arm_add_f32(&pid->error[0], &pid->integral, &pid->integral, 1);
		LimitMax(pid->integral, pid->max_iout);
		
		//分别计算PID
		//output = kp * Error + ki * Integral + kd * DDError
		
    //pid->Pout = pid->Kp * pid->error[0];
		arm_mult_f32(&pid->Kp, &pid->error[0], &pid->Pout, 1);
    //pid->Iout = pid->Ki * pid->integral;
		arm_mult_f32(&pid->Ki, &pid->integral, &pid->Iout, 1);
		
		//pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[1], 1);
		arm_sub_f32(&pid->error[1], &pid->error[2], &pid->Dbuf[2], 1);
		arm_sub_f32(&pid->Dbuf[2], &pid->Dbuf[1], &pid->Dbuf[0], 1);
		
		//pid->Dout = pid->Kd * pid->Dbuf[0];
		arm_sub_f32(&pid->Kd, &pid->Dbuf[0], &pid->Dout, 1);
		
		//pid->out = pid->Pout + pid->Iout + pid->Dout;
		arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
		arm_add_f32(&PIout, &pid->Dout, &pid->out, 1);
		
		LimitMax(pid->out, pid->max_out);
    
    return pid->out;
}

