#include "chassis.h"

drv_can_t				  *chas_drv[CHAS_MOTOR_CNT];
motor_3508_t			*chas_motor[CHAS_MOTOR_CNT];
motor_3508_info_t	*chas_motor_info[CHAS_MOTOR_CNT];
uint16_t           out[CHAS_MOTOR_CNT];

void Chassis_Init(void);
void Chassis_Ctrl(void);
void Chassis_Test(void);
void Chassis_SelfProtect(void);

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
	.dev = &chas_dev,
	.info = &chas_info,
	.init = Chassis_Init,
	.test = Chassis_Test,
	.ctrl = Chassis_Ctrl,
	.self_protect = Chassis_SelfProtect,
};

void Chassis_Init(void)
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


//底盘电机卸力
static void Chassis_Stop(chassis_dev_t *chas_dev)
{
	for(uint8_t i = 0; i < CHAS_MOTOR_CNT; i++)
	{
		chas_dev->chas_motor[i]->pid->speed_out = 0.0f;
		out[i] = (int16_t)chas_dev->chas_motor[i]->pid->speed_out;
	}
	can1_tx_buf[(chas_motor[CHAS_LF]->driver->rx_id - 0x201) * 2] = (out[CHAS_LF] >> 8) & 0xFF;
	can1_tx_buf[(chas_motor[CHAS_LF]->driver->rx_id - 0x201) * 2 + 1] = (out[CHAS_LF] & 0xFF);
	can1_tx_buf[(chas_motor[CHAS_RF]->driver->rx_id - 0x201) * 2] = (out[CHAS_RF] >> 8) & 0xFF;
	can1_tx_buf[(chas_motor[CHAS_RF]->driver->rx_id - 0x201)* 2 + 1] = (out[CHAS_RF] & 0xFF);
	can1_tx_buf[(chas_motor[CHAS_LB]->driver->rx_id - 0x201) * 2] = (out[CHAS_LB] >> 8) & 0xFF;
	can1_tx_buf[(chas_motor[CHAS_LB]->driver->rx_id - 0x201) * 2 + 1] = (out[CHAS_LB] & 0xFF);
	can1_tx_buf[(chas_motor[CHAS_RB]->driver->rx_id - 0x201) * 2] = (out[CHAS_RB] >> 8) & 0xFF;
	can1_tx_buf[(chas_motor[CHAS_RB]->driver->rx_id - 0x201) * 2 + 1] = (out[CHAS_RB] & 0xFF);
}

//底盘电机速度环
static void Chassis_Speed_PidCalc(chassis_dev_t *chas_dev, chassis_motor_cnt_t MOTORx)
{
		chas_motor[MOTORx]->pid->speed_out = PID_Inc_Calc(&chas_motor[MOTORx]->pid->speed, chas_motor[MOTORx]->info->speed_rpm,
                                                 chas_motor[MOTORx]->pid->speed.set);
		out[MOTORx] = (int16_t)chas_motor[MOTORx]->pid->speed_out;
//	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
//	single_pid_ctrl(&pid[MOTORx].speed);
//	pid[MOTORx].out = pid[MOTORx].speed.out;
}

//底盘电机PID输出
static void Chassis_PidOut(chassis_dev_t *chas_dev)
{
	for(uint8_t i = 0; i < CHAS_MOTOR_CNT; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) 
		{
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2] = (out[CHAS_LF] >> 8) & 0xFF;
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2 + 1] = (out[CHAS_LF] & 0xFF);
//         chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].speed_out);
		} 
		else 
		{
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2] = 0;
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2 + 1] = 0;
//         chas_drv[i]->add_halfword(chas_drv[i], 0);
		}
	}    
}

void Chassis_PidCtrl(void)
{
	// 底盘电机速度环
	Chassis_Speed_PidCalc(&chas_dev, CHAS_LF);
//	Chassis_Speed_PidCalc(chas_motor_pid, CHAS_LB);
//	Chassis_Speed_PidCalc(chas_motor_pid, CHAS_RF);
//	Chassis_Speed_PidCalc(chas_motor_pid, CHAS_RB);
	
	// 底盘电机输出响应
	Chassis_PidOut(&chas_dev);
}

void Chassis_SelfProtect(void)
{
	Chassis_Stop(&chas_dev);
//	Chassis_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
//	Chassis_GetInfo();
}

void Chassis_GetRcInfo(void)
{
	chas_motor[CHAS_LF]->pid->speed.set = (float)rc_sensor.info->ch0;
}

void Chassis_GetInfo(void)
{
	Chassis_GetRcInfo();
}

void Chassis_Ctrl(void)
{
//	/*----信息读入----*/
	Chassis_GetInfo();
//	/*----期望修改----*/ 
//	if(chas_info.remote_mode == RC) {
//		Chassis_RcCtrl();
//	}
//	else if(chas_info.remote_mode == KEY) {
//		Chassis_KeyCtrl();
//	}
//	/*----最终输出----*/
	Chassis_PidCtrl();
}

void Chassis_Test(void)
{
    
}

