#include "gimbal.h"

// 子结构体（便于编写与查看）   0:yaw轴  1:pitch轴
drv_can_t				  *gim_drv[2];
motor_6020_t			*gim_motor[2];
motor_6020_info_t	*gim_motor_info[2];
int16_t            gim_out[2];

float yaw_PID[6] = {11.0f, 1.2f, 0.f, 8.f, 0.f, 0.f};
float pitch_PID[6] = {20.0f, 1.3f, 0.0f, 8.0f, 0.0f, 0.0f};

void Gimbal_Init(void);
void Gimbal_Ctrl(void);
void Gimbal_SelfProtect(void);

float speed_output;
float speed_target;

// 云台设备
gimbal_dev_t		gim_dev = {
	.yaw_motor = &yaw_motor,
	.pitch_motor = &pitch_motor,
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 云台模块信息
gimbal_info_t 	gim_info = {
	.remote_mode = RC,
	.yaw_mode = G_Y_machine,
	.pitch_mode = G_P_machine,
};

gimbal_conf_t   gim_conf = {
	.restart_yaw_imu_angle = 0.0f,
	.restart_pitch_imu_angle = 0.0f,
	.MID_VALUE = 4777,
	.restart_yaw_motor_angle = 4777, // 双枪 2100  麦轮 4777
	.restart_pitch_motor_angle = 6900, // 双枪 6600  麦轮 6900
	.rc_pitch_motor_offset = 110,
	.rc_yaw_imu_offset = 3300.0f,
	.rc_pitch_imu_offset = 1100.0f,
	.max_pitch_imu_angle = 15.0f,
	.min_pitch_imu_angle = -25.0f,
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

void PID_ParamsInit(motor_6020_t *motor)
{
	if (motor->id == DEV_ID_GIMBAL_YAW)
	{
		if (gimbal.info->yaw_mode == G_Y_machine)
		{
			motor->pid->speed.Kp = yaw_PID[0];//YAW_MACHINE_SP_KP;
			motor->pid->speed.Ki = yaw_PID[1];//YAW_MACHINE_SP_KI;
			motor->pid->speed.Kd = yaw_PID[2];//YAW_MACHINE_SP_KD;
			motor->pid->angle.Kp = yaw_PID[3];//YAW_MACHINE_AG_KP;
			motor->pid->angle.Ki = yaw_PID[4];//YAW_MACHINE_AG_KI;
			motor->pid->angle.Kd = yaw_PID[5];//YAW_MACHINE_AG_KD;
		}
		else if (gimbal.info->yaw_mode == G_Y_gyro)
		{
			motor->pid->speed.Kp = YAW_GYRO_SP_KP;
			motor->pid->speed.Ki = YAW_GYRO_SP_KI;
			motor->pid->speed.Kd = YAW_GYRO_SP_KD;
			motor->pid->angle.Kp = YAW_GYRO_AG_KP;
			motor->pid->angle.Ki = YAW_GYRO_AG_KI;
			motor->pid->angle.Kd = YAW_GYRO_AG_KD;
		}
	}
	else if (motor->id == DEV_ID_GIMBAL_PITCH)
	{
		if (gimbal.info->pitch_mode == G_P_machine)
		{
			motor->pid->speed.Kp = pitch_PID[0];//PITCH_MACHINE_SP_KP;
			motor->pid->speed.Ki = pitch_PID[1];//PITCH_MACHINE_SP_KI;
			motor->pid->speed.Kd = pitch_PID[2];//PITCH_MACHINE_SP_KD;
			motor->pid->angle.Kp = pitch_PID[3];//PITCH_MACHINE_AG_KP;
			motor->pid->angle.Ki = pitch_PID[4];//PITCH_MACHINE_AG_KI;
			motor->pid->angle.Kd = pitch_PID[5];//PITCH_MACHINE_AG_KD;
		}
		else if (gimbal.info->pitch_mode == G_P_gyro)
		{
			motor->pid->speed.Kp = PITCH_GYRO_SP_KP;
			motor->pid->speed.Ki = PITCH_GYRO_SP_KI;
			motor->pid->speed.Kd = PITCH_GYRO_SP_KD;
			motor->pid->angle.Kp = PITCH_GYRO_AG_KP;
			motor->pid->angle.Ki = PITCH_GYRO_AG_KI;
			motor->pid->angle.Kd = PITCH_GYRO_AG_KD;
		}
	}
}

void Gimbal_GetBaseInfo(void)
{
	gimbal.info->measure_yaw_motor_angle = yaw_motor.info->ecd;
	gimbal.info->measure_yaw_motor_speed = yaw_motor.info->speed_rpm;
	gimbal.info->measure_pitch_motor_angle = pitch_motor.info->ecd;
	gimbal.info->measure_pitch_motor_speed = pitch_motor.info->speed_rpm;
	
	gimbal.info->measure_yaw_imu_speed = imu_sensor.info->rate_yaw;
	gimbal.info->measure_yaw_imu_angle = imu_sensor.info->yaw;
	gimbal.info->measure_pitch_imu_speed = imu_sensor.info->rate_pitch;
	gimbal.info->measure_pitch_imu_angle = imu_sensor.info->pitch;
	gimbal.info->measure_roll_imu_speed = imu_sensor.info->rate_roll;
	gimbal.info->measure_roll_imu_angle = imu_sensor.info->roll;
	
	gimbal.info->yaw_real_rate = gimbal.info->measure_yaw_imu_speed * arm_cos_f32(gimbal.info->measure_pitch_imu_angle * ANGLE_TO_RAD) \
	                           + gimbal.info->measure_roll_imu_speed * arm_sin_f32(gimbal.info->measure_pitch_imu_angle * ANGLE_TO_RAD);
	gimbal.info->pitch_real_rate = gimbal.info->measure_pitch_imu_speed * arm_cos_f32(gimbal.info->measure_roll_imu_angle * ANGLE_TO_RAD) \
	                             + gimbal.info->measure_roll_imu_speed * arm_sin_f32(gimbal.info->measure_roll_imu_angle * ANGLE_TO_RAD);
	
}

void Gimbal_GetRcInfo(void)
{
	if (gimbal.info->pitch_mode == G_P_machine)
	{
		gimbal.info->target_pitch_motor_deltaangle = rc_sensor.info->ch1 / gim_conf.rc_pitch_motor_offset;
		gimbal.info->target_pitch_imu_angle = gimbal.info->measure_pitch_imu_angle;
		gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	}
	else if (gimbal.info->pitch_mode == G_P_gyro)
	{
		gimbal.info->target_pitch_motor_angle = gimbal.info->measure_pitch_motor_angle;
		gimbal.info->target_pitch_imu_deltaangle = (float)rc_sensor.info->ch1 / gim_conf.rc_pitch_imu_offset;
		gimbal.info->target_yaw_imu_deltaangle = -(float)rc_sensor.info->ch0 / gim_conf.rc_pitch_imu_offset;
		
	}
	else
	{
		gimbal.info->target_pitch_motor_angle = gimbal.info->measure_pitch_motor_angle;
		gimbal.info->target_pitch_imu_angle = gimbal.info->measure_pitch_imu_angle;
		gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	}
}

void Gimbal_GetKeyInfo(void)
{
	
}

void Gimbal_GetInfo(void)
{
	Gimbal_GetBaseInfo();
	
	PID_ParamsInit(&yaw_motor);
	PID_ParamsInit(&pitch_motor);
	
	if (flag.gimbal_flag.reset_ok == 0)
	{
		if (RC_IsChannelReset() && flag.gimbal_flag.reset_start == 0)
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

void Gimbal_MotoReset(void)
{
	gimbal.info->target_pitch_motor_angle = gim_conf.restart_pitch_motor_angle;
	gimbal.info->target_yaw_motor_angle = gim_conf.restart_yaw_motor_angle;
	
	flag.gimbal_flag.reset_start = 0;
}

void Gimbal_GyroReset(void)
{
	gimbal.info->target_pitch_imu_angle = gim_conf.restart_pitch_imu_angle;
	gimbal.info->target_yaw_imu_angle = gimbal.info->measure_yaw_imu_angle;
	
	flag.gimbal_flag.reset_start = 0;
}

void Gimbal_Reset(void)
{
	if (flag.gimbal_flag.reset_start == 1 && flag.gimbal_flag.reset_ok == 0)
	{
		if (gimbal.info->yaw_mode == G_Y_machine)
		{
			Gimbal_MotoReset();
		}
		else if (gimbal.info->yaw_mode == G_Y_gyro)
		{
			Gimbal_GyroReset();
		}
	}
}

void Gimbal_RcCtrl(void)
{
	// 机械模式
	if (gimbal.info->yaw_mode == G_Y_machine)
	{
			gimbal.info->target_pitch_motor_angle += gimbal.info->target_pitch_motor_deltaangle;
			
			// pitch轴电机限位
			if (gimbal.info->target_pitch_motor_angle > gim_conf.max_pitch_motor_angle)
			{
				gimbal.info->target_pitch_motor_angle = gim_conf.max_pitch_motor_angle;
			}
			else if (gimbal.info->target_pitch_motor_angle < gim_conf.min_pitch_motor_angle)
			{
				gimbal.info->target_pitch_motor_angle = gim_conf.min_pitch_motor_angle;
			}
		
			
			// yaw轴电机error过零点处理
		  gimbal.info->yaw_motor_angle_err = gim_conf.restart_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle;
			if (gimbal.info->yaw_motor_angle_err > HALF_ECD_RANGE)
			{
				gimbal.info->yaw_motor_angle_err -= ECD_RANGE;
			}
			else if (gimbal.info->yaw_motor_angle_err < -HALF_ECD_RANGE)
			{
				gimbal.info->yaw_motor_angle_err += ECD_RANGE;
			}
			// yaw轴电机就近归中
			if (abs(gimbal.info->yaw_motor_angle_err) > (HALF_ECD_RANGE / 2))
			{
				if (gimbal.conf->restart_yaw_motor_angle > HALF_ECD_RANGE)
				{
					gimbal.conf->restart_yaw_motor_angle = gimbal.conf->MID_VALUE - HALF_ECD_RANGE;
					flag.chassis_flag.forward = 0;
				}
				else
				{
					gimbal.conf->restart_yaw_motor_angle = gimbal.conf->MID_VALUE;
					flag.chassis_flag.forward = 1;
				}
			}
			gim_info.target_yaw_motor_angle = gim_conf.restart_yaw_motor_angle;
			
	}
	// 陀螺仪模式
	else if (gimbal.info->yaw_mode == G_Y_gyro)
	{
			gimbal.info->target_pitch_imu_angle += gimbal.info->target_pitch_imu_deltaangle;
			gimbal.info->target_yaw_imu_angle += gimbal.info->target_yaw_imu_deltaangle;
			
			
			// pitch轴电机限位（陀螺仪模式下需要修改）
			if (gimbal.info->target_pitch_imu_angle > gim_conf.max_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.max_pitch_imu_angle;
			}
			else if (gimbal.info->target_pitch_imu_angle < gim_conf.min_pitch_imu_angle)
			{
				gimbal.info->target_pitch_imu_angle = gim_conf.min_pitch_imu_angle;
			}
			
			
			// yaw轴电机target过零点处理
			if (gimbal.info->target_yaw_imu_angle > 180.0f)
			{
				gimbal.info->target_yaw_imu_angle -= 360.0f;
			}
			else if (gimbal.info->target_yaw_imu_angle < -180.0f)
			{
				gimbal.info->target_yaw_imu_angle += 360.0f;
			}
	}
	else
	{
			Gimbal_Stop();
	}
}

void Gimbal_KeyCtrl(void)
{
	
}

void Gimbal_Yaw_Angle_PidCalc(motor_6020_t *motor)
{
	if (gimbal.info->yaw_mode == G_Y_machine)
	{
		// yaw轴电机error过零点处理
		if (gimbal.info->target_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle > HALF_ECD_RANGE)
		{
			gimbal.info->measure_yaw_motor_angle += ECD_RANGE;
		}
		else if (gimbal.info->target_yaw_motor_angle - gimbal.info->measure_yaw_motor_angle < -HALF_ECD_RANGE)
		{
			gimbal.info->measure_yaw_motor_angle -= ECD_RANGE;
		}
		motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, (float)gimbal.info->measure_yaw_motor_angle, (float)gimbal.info->target_yaw_motor_angle);
		motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, gimbal.info->measure_yaw_imu_speed, motor->pid->angle_out);
	}
	else if (gimbal.info->yaw_mode == G_Y_gyro)
	{
		// yaw轴电机error过零点处理
		if (gimbal.info->target_yaw_imu_angle - gimbal.info->measure_yaw_imu_angle > 180.0f)
		{
			gimbal.info->measure_yaw_imu_angle += 360.0f;
		}
		else if (gimbal.info->target_yaw_imu_angle - gimbal.info->measure_yaw_imu_angle < -180.0f)
		{
			gimbal.info->measure_yaw_imu_angle -= 360.0f;
		}
		motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, gimbal.info->measure_yaw_imu_angle, gimbal.info->target_yaw_imu_angle);
		motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, gimbal.info->measure_yaw_imu_speed, motor->pid->angle_out);
		speed_output = motor->pid->speed_out;
		speed_target = motor->pid->angle_out;
	}
	
	gim_out[motor->driver->rx_id - 0x205] = (int16_t)motor->pid->speed_out;
}

void Gimbal_Pitch_Angle_PidCalc(motor_6020_t *motor)
{
	if (gimbal.info->pitch_mode == G_P_machine)
	{
		motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, (float)gimbal.info->measure_pitch_motor_angle, (float)gimbal.info->target_pitch_motor_angle);
		motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, gimbal.info->measure_pitch_imu_speed, motor->pid->angle_out);
	}
	else if (gimbal.info->pitch_mode == G_P_gyro)
	{
		motor->pid->angle_out = PID_Plc_Calc(&motor->pid->angle, gimbal.info->measure_pitch_imu_angle, gimbal.info->target_pitch_imu_angle);
		motor->pid->speed_out = PID_Plc_Calc(&motor->pid->speed, gimbal.info->measure_pitch_imu_speed, motor->pid->angle_out);
	}
	
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
	Gimbal_Pitch_Angle_PidCalc(&pitch_motor);
	
	Gimbal_SendPidOut();
}

void Gimbal_Ctrl(void)
{
	Gimbal_GetInfo();
	
	Gimbal_Reset();
	
	if (gim_info.remote_mode == RC)
	{
		Gimbal_RcCtrl();
	}
//	else if(gim_info.remote_mode == KEY) {
//		Gimbal_KeyCtrl();
//	}
	Gimbal_PidCtrl();
}


