#include "gimbal.h"

// 子结构体（便于编写与查看）   0:yaw轴  1:pitch轴
drv_can_t				  *gim_drv[2];
motor_6020_t			*gim_motor[2];
motor_6020_info_t	*gim_motor_info[2];
int16_t            gim_out[2];
int16_t set = 110;
int8_t deadare = 0;

void Gimbal_Init(void);
void Gimbal_Ctrl(void);
void Gimbal_SelfProtect(void);

// 底盘模块传感器
gimbal_dev_t		gim_dev = {
	.yaw_motor = &yaw_motor,
	.pitch_motor = &pitch_motor,
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
gimbal_info_t 	gim_info = {
	.remote_mode = RC,
	.yaw_mode = G_Y_follow,
	.pitch_mode = G_P_machine,
};

gimbal_conf_t   gim_conf = {
	.restart_yaw_motor_angle = 4777, // 双枪 2100  麦轮 4777
	.restart_pitch_motor_angle = 6900, // 双枪 6600  麦轮 6900
	
	.max_pitch_motor_angle = 7200, // 双枪、麦轮 7200  
	.min_pitch_motor_angle = 6100, // 双枪、麦轮 6100
};

gimbal_t gimbal = {
	.dev = &gim_dev,
	.info = &gim_info,
	.conf = &gim_conf,
	.init = Gimbal_Init,
	.ctrl = Gimbal_Ctrl,
	.self_protect = Gimbal_SelfProtect,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Gimbal_Motor_Init(void)
{
	yaw_motor.init(&yaw_motor);
	pitch_motor.init(&pitch_motor);
	
	pitch_motor.pid->angle.set = 4000.0f;
}

void Gimbal_Init(void)
{
	gim_motor[0] = gimbal.dev->yaw_motor;
	gim_motor[1] = gimbal.dev->pitch_motor;
	gim_motor_info[0] = gimbal.dev->yaw_motor->info;
	gim_motor_info[1] = gimbal.dev->pitch_motor->info;
	gim_drv[0] = gimbal.dev->yaw_motor->driver;
	gim_drv[1] = gimbal.dev->pitch_motor->driver;
	
	Gimbal_Motor_Init();
}

void Gimbal_GetBaseInfo(void)
{
	gimbal.info->measure_pitch_motor_angle = pitch_motor.info->total_ecd;
	gimbal.info->measure_yaw_motor_angle = yaw_motor.info->total_ecd;
	
}

void Gimbal_GetRcInfo(void)
{
	gimbal.info->target_pitch_motor_deltaangle = rc_sensor.info->ch1 / set;
	
}

void Gimbal_GetKeyInfo(void)
{
	
}

void Gimbal_GetInfo(void)
{
	Gimbal_GetBaseInfo();
	if (flag.gimbal_flag.reset_ok == 0)
	{
		if (rc_sensor.info->ch0 == 0 \
			&& rc_sensor.info->ch1 == 0 \
			&& rc_sensor.info->ch2 == 0 \
			&& rc_sensor.info->ch3 == 0 \
		  && flag.gimbal_flag.reset_start == 0)
		{
			flag.gimbal_flag.reset_ok = 1;
		}
	}
	else
	{
		if(gim_info.remote_mode == RC) {
			Gimbal_GetRcInfo();
		}
		else if(gim_info.remote_mode == KEY) {
			Gimbal_GetKeyInfo();
		}
	}
}

void Gimbal_Stop()
{
	gim_motor[0]->pid->speed_out = 0.0f;
	gim_motor[1]->pid->speed_out = 0.0f;
	gim_out[0] = (int16_t)gim_motor[0]->pid->speed_out;
	gim_out[1] = (int16_t)gim_motor[1]->pid->speed_out;
	
	can1_tx_buf[8] = (gim_out[0] >> 8) & 0xFF;
	can1_tx_buf[9] = gim_out[0] & 0xFF;
	can1_tx_buf[10] = (gim_out[1] >> 8) & 0xFF;
	can1_tx_buf[11] = gim_out[1] & 0xFF;
}

void Gimbal_SelfProtect(void)
{
	Gimbal_Stop();
	Gimbal_GetInfo();
	flag.gimbal_flag.reset_start = 1;
	flag.gimbal_flag.reset_ok = 0;
}

void Gimbal_Yaw_Angle_PidCalc(motor_6020_t *motor)
{
	motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, (float)motor->info->total_ecd, motor->pid->angle.set);
	motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, (float)motor->info->speed_rpm, motor->pid->angle_out);
	
	gim_out[motor->driver->rx_id - 0x205] = (int16_t)motor->pid->speed_out;
}

void Gimbal_Pitch_Angle_PidCalc(motor_6020_t *motor)
{
	motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, (float)motor->info->ecd, motor->pid->angle.set);
//	motor->pid->speed_out = PID_Inc_Calc(&motor->pid->speed, (float)motor->info->speed_rpm, motor->pid->angle_out);
	motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, (float)motor->info->speed_rpm, motor->pid->angle_out);
	
	gim_out[motor->driver->rx_id - 0x205] = (int16_t)motor->pid->speed_out;
}

void Gimbal_SendPidOut(void)
{
	// yaw轴电机离线保护
	if (yaw_motor.work_state == DEV_ONLINE)
	{
		can1_tx_buf[8] = (gim_out[yaw_motor.driver->rx_id - 0x205] >> 8) & 0xFF;
		can1_tx_buf[9] = gim_out[yaw_motor.driver->rx_id - 0x205] & 0xFF;
	}
	else
	{
		can1_tx_buf[8] = 0;
		can1_tx_buf[9] = 0;
	}
	
	// pitch轴电机离线保护
	if (pitch_motor.work_state == DEV_ONLINE)
	{
		can1_tx_buf[10] = (gim_out[pitch_motor.driver->rx_id - 0x205] >> 8) & 0xFF;
		can1_tx_buf[11] = gim_out[pitch_motor.driver->rx_id - 0x205] & 0xFF;
	}
	else
	{
		can1_tx_buf[10] = 0;
		can1_tx_buf[11] = 0;
	}
}

void Gimbal_PidCtrl(void)
{
	Gimbal_Yaw_Angle_PidCalc(&yaw_motor);
	
//	if ((gimbal.info->target_pitch_motor_angle - gimbal.info->measure_pitch_motor_angle < deadare)
//	 && (gimbal.info->target_pitch_motor_angle - gimbal.info->measure_pitch_motor_angle > -deadare))
//	{
//		gim_out[pitch_motor.driver->rx_id - 0x205] = pitch_motor.pid->speed_out;
//	}
//	else
//	{
		Gimbal_Pitch_Angle_PidCalc(&pitch_motor);
//	}
	
	Gimbal_SendPidOut();
}

void Gimbal_RcCtrl(void)
{
	if (gimbal.info->yaw_mode == G_Y_follow)
	{
		gimbal.info->target_pitch_motor_angle += gimbal.info->target_pitch_motor_deltaangle;
		
		yaw_motor.pid->angle.set = (float)gimbal.info->target_yaw_motor_angle;
		
		//pitch轴电机限位
		if (gimbal.info->target_pitch_motor_angle > gim_conf.max_pitch_motor_angle)
		{
			gimbal.info->target_pitch_motor_angle = gim_conf.max_pitch_motor_angle;
		}
		else if (gimbal.info->target_pitch_motor_angle < gim_conf.min_pitch_motor_angle)
		{
			gimbal.info->target_pitch_motor_angle = gim_conf.min_pitch_motor_angle;
		}
		pitch_motor.pid->angle.set = (float)gimbal.info->target_pitch_motor_angle;
	}
	else
	{
		Gimbal_Stop();
	}
}

void Gimbal_Reset(void)
{
	if (flag.gimbal_flag.reset_start == 1 && flag.gimbal_flag.reset_ok == 0)
	{
		gimbal.info->target_yaw_motor_deltaangle = gim_conf.restart_yaw_motor_angle - yaw_motor.info->ecd;
		gimbal.info->target_pitch_motor_angle = gim_conf.restart_pitch_motor_angle;
		
		if (gimbal.info->target_yaw_motor_deltaangle > HALF_ECD_RANGE)
		{
			gimbal.info->target_yaw_motor_deltaangle -= ECD_RANGE;
		}
		gimbal.info->target_yaw_motor_angle = yaw_motor.info->total_ecd + gimbal.info->target_yaw_motor_deltaangle;
		
		flag.gimbal_flag.reset_start = 0;
	}
}

void Gimbal_Ctrl(void)
{
	Gimbal_GetInfo();
	
	Gimbal_Reset();
	
	if (gim_info.remote_mode == RC)
	{
		Gimbal_RcCtrl();
	}
	
	Gimbal_PidCtrl();
}


