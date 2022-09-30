#include "chassis.h"

drv_can_t				*chas_drv[CHAS_MOTOR_CNT];
chassis_motor_t			*chas_motor[CHAS_MOTOR_CNT];
chassis_motor_info_t	*chas_motor_info[CHAS_MOTOR_CNT];

void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);

// 底盘电机PID控制器(Plc-Inc)
chassis_motor_pid_t 	chas_motor_pid[CHAS_MOTOR_CNT] = {
	[CHAS_LF] = {
		.speed.Kp = SPEED_KP,
		.speed.Ki = SPEED_KI,
		.speed.Kd = SPEED_KD,
		.speed.max_iout = SP_MAX_I_OUT,
		.speed.max_out = SP_MAX_OUT,
		.angle.Kp = ANGLE_KP,
		.angle.Ki = ANGLE_KI,
		.angle.Kd = ANGLE_KD,
		.angle.max_integral = AG_MAX_INTEGRAL,
		.angle.max_out = AG_MAX_OUT,
	},
	[CHAS_RF] = {
		.speed.Kp = SPEED_KP,
		.speed.Ki = SPEED_KI,
		.speed.Kd = SPEED_KD,
		.speed.max_iout = SP_MAX_I_OUT,
		.speed.max_out = SP_MAX_OUT,
		.angle.Kp = ANGLE_KP,
		.angle.Ki = ANGLE_KI,
		.angle.Kd = ANGLE_KD,
		.angle.max_integral = AG_MAX_INTEGRAL,
		.angle.max_out = AG_MAX_OUT,
	},
	[CHAS_LB] = {
		.speed.Kp = SPEED_KP,
		.speed.Ki = SPEED_KI,
		.speed.Kd = SPEED_KD,
		.speed.max_iout = SP_MAX_I_OUT,
		.speed.max_out = SP_MAX_OUT,
		.angle.Kp = ANGLE_KP,
		.angle.Ki = ANGLE_KI,
		.angle.Kd = ANGLE_KD,
		.angle.max_integral = AG_MAX_INTEGRAL,
		.angle.max_out = AG_MAX_OUT,
	},
	[CHAS_RB] = {
		.speed.Kp = SPEED_KP,
		.speed.Ki = SPEED_KI,
		.speed.Kd = SPEED_KD,
		.speed.max_iout = SP_MAX_I_OUT,
		.speed.max_out = SP_MAX_OUT,
		.angle.Kp = ANGLE_KP,
		.angle.Ki = ANGLE_KI,
		.angle.Kd = ANGLE_KD,
		.angle.max_integral = AG_MAX_INTEGRAL,
		.angle.max_out = AG_MAX_OUT,
	},
};

// 底盘模块控制器
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
};

// 底盘模块传感器
chassis_dev_t		chas_dev = {
	.chas_motor[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_motor[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_motor[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_motor[CHAS_RB] = &chassis_motor[CHAS_RB],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.local_mode = CHASSIS_MODE_NORMAL,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
};

void CHASSIS_Init(void)
{
	chas_drv[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->driver;
	chas_drv[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->driver;
	chas_drv[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->driver;
	chas_drv[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->driver;

	chas_motor[CHAS_LF] = chas_dev.chas_motor[CHAS_LF];
	chas_motor[CHAS_RF] = chas_dev.chas_motor[CHAS_RF];
	chas_motor[CHAS_LB] = chas_dev.chas_motor[CHAS_LB];
	chas_motor[CHAS_RB] = chas_dev.chas_motor[CHAS_RB];	
	
	chas_motor_info[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->info;
	chas_motor_info[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->info;
	chas_motor_info[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->info;
	chas_motor_info[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->info;
	
}


//底盘电机PID参数初始化
void CHASSIS_PidParamsInit(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		PID_Init(&pid[i].speed);
	}
}

//底盘电机卸力
static void CHASSIS_Stop(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHAS_MOTOR_CNT; i++)
	{
		pid[i].speed_out = 0;
//		chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
	}
}

//底盘电机PID输出
static void CHASSIS_PidOut(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<1; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) 
		{
         chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].speed_out);
		} 
		else 
		{
         chas_drv[i]->add_halfword(chas_drv[i], 0);
		}
	}    
}

//底盘电机速度环
static void CHASSIS_Speed_PidCalc(chassis_motor_pid_t *pid, chassis_dev_t *dev, chassis_motor_cnt_t MOTORx)
{
//	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
//	single_pid_ctrl(&pid[MOTORx].speed);
//	pid[MOTORx].out = pid[MOTORx].speed.out;
}

void CHASSIS_SelfProtect(void)
{
//	CHASSIS_Stop(chas_motor_pid);
//	CHASSIS_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
//	CHASSIS_GetInfo();
}

void CHASSIS_PidCtrl(void)
{
	// 底盘电机速度环
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LB);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RB);
	
	// 底盘电机输出响应
	CHASSIS_PidOut(chas_motor_pid);
}

void CHASSIS_Ctrl(void)
{
//	/*----信息读入----*/
//	CHASSIS_GetInfo();
//	/*----期望修改----*/ 
//	if(chas_info.remote_mode == RC) {
//		CHASSIS_RcCtrl();
//	}
//	else if(chas_info.remote_mode == KEY) {
//		CHASSIS_KeyCtrl();
//	}
//	/*----最终输出----*/
//	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
    
}

