#include "shoot.h"

int16_t    shoot_out[3];

void Shoot_Init(void);
void Shoot_Ctrl(void);
void Shoot_SelfProtect(void);

// 发射机构设备
shoot_dev_t		shoot_dev = {
	.shoot_left = &shoot_motor[SHOOT_L],
	.shoot_right = &shoot_motor[SHOOT_R],
	.dial_motor = &dial_motor,
	.rc_sensor = &rc_sensor,
};

// 发射机构信息
shoot_info_t 	shoot_info = {
	.remote_mode = RC,
};

shoot_conf_t   shoot_conf = {
};

launcher_t launcher = {
	.dev = &shoot_dev,
	.info = &shoot_info,
	.conf = &shoot_conf,
	.init = Shoot_Init,
	.ctrl = Shoot_Ctrl,
	.self_protect = Shoot_SelfProtect,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Shoot_Init(void)
{
	shoot_motor[SHOOT_L].init(&shoot_motor[SHOOT_L]);
	shoot_motor[SHOOT_R].init(&shoot_motor[SHOOT_R]);
	dial_motor.init(&dial_motor);
}

void Shoot_GetBaseInfo(void)
{
	launcher.info->measure_left_speed = shoot_motor[SHOOT_L].info->speed_rpm;
	launcher.info->measure_right_speed = shoot_motor[SHOOT_R].info->speed_rpm;
	launcher.info->measure_dial_speed = dial_motor.info->speed_rpm;
	launcher.info->measure_dial_angle = dial_motor.info->angle;
	
}

void Shoot_GetLauncherState(void)
{
	
}

void Shoot_GetRcState(void)
{
	if (gimbal.info->gimbal_mode == gim_gyro)
	{
		if (rc_sensor.info->s2 == 3)
		{
			launcher.info->launcher_mode = Shoot_Reset;
		}
//		else if ()
	}
	else if (gimbal.info->gimbal_mode == gim_machine)
	{
		
	}
}

void Shoot_GetCtrlInfo(void)
{
	Shoot_GetLauncherState();
	if (launcher.info->remote_mode == RC)
	{
		Shoot_GetRcState();
	}
//	else if (launcher.info->remote_mode == KEY)
//	{
//		
//	}
}

void Shoot_GetInfo(void)
{
	Shoot_GetBaseInfo();
	
	Shoot_GetCtrlInfo();
}

void Shoot_Speed_PidCalc(motor_3508_t *motor)
{
	
}

void Dial_Angle_PidCalc(motor_2006_t *motor)
{
	
}

void Shoot_SendPidOut(void)
{
	// 摩擦轮电机离线保护
	if (shoot_motor[SHOOT_L].work_state == DEV_ONLINE)
	{
		can2_tx_buf[0] = (shoot_out[shoot_motor[SHOOT_L].driver->rx_id - 0x200] >> 8) & 0xFF;
		can2_tx_buf[1] = shoot_out[shoot_motor[SHOOT_L].driver->rx_id - 0x200] & 0xFF;
	}
	else
	{
		can2_tx_buf[0] = 0;
		can2_tx_buf[1] = 0;
	}
	
	if (shoot_motor[SHOOT_R].work_state == DEV_ONLINE)
	{
		can2_tx_buf[2] = (shoot_out[shoot_motor[SHOOT_R].driver->rx_id - 0x200] >> 8) & 0xFF;
		can2_tx_buf[3] = shoot_out[shoot_motor[SHOOT_R].driver->rx_id - 0x200] & 0xFF;
	}
	else
	{
		can2_tx_buf[2] = 0;
		can2_tx_buf[3] = 0;
	}
	
	// 拨盘电机离线保护
	if (dial_motor.work_state == DEV_ONLINE)
	{
		can2_tx_buf[4] = (shoot_out[dial_motor.driver->rx_id - 0x200] >> 8) & 0xFF;
		can2_tx_buf[5] = shoot_out[dial_motor.driver->rx_id - 0x200] & 0xFF;
	}
	else
	{
		can2_tx_buf[4] = 0;
		can2_tx_buf[5] = 0;
	}
}

void Shoot_PidCtrl(void)
{
	Shoot_Speed_PidCalc(shoot_motor);
	Dial_Angle_PidCalc(&dial_motor);
	
	Shoot_SendPidOut();
}

void Launcher_Ctrl(void)
{
	
}

void Shoot_Ctrl(void)
{
	Shoot_GetInfo();
	
	Launcher_Ctrl();
	
	
}

// 离线保护
void Shoot_Stop(void)
{
	shoot_motor[SHOOT_L]->pid->speed_out = 0.0f;
	shoot_motor[SHOOT_R]->pid->speed_out = 0.0f;
	dial_motor.pid->speed_out = 0.0f;
	shoot_out[0] = (int16_t)shoot_motor[SHOOT_L]->pid->speed_out;
	shoot_out[1] = (int16_t)shoot_motor[SHOOT_R]->pid->speed_out;
	shoot_out[2] = (int16_t)dial_motor.pid->speed_out;
	
	can2_tx_buf[0] = (shoot_out[0] >> 8) & 0xFF;
	can2_tx_buf[1] = shoot_out[0] & 0xFF;
	can2_tx_buf[2] = (shoot_out[1] >> 8) & 0xFF;
	can2_tx_buf[3] = shoot_out[1] & 0xFF;
	can2_tx_buf[4] = (shoot_out[2] >> 8) & 0xFF;
	can2_tx_buf[5] = shoot_out[2] & 0xFF;
}

void Shoot_SelfProtect(void)
{
	Shoot_Stop();
	Shoot_GetInfo();
}

