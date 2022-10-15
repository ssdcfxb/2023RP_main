#include "shoot.h"

int16_t    shoot_out[3];

void Shoot_Init(void);
void Shoot_Ctrl(void);
void Shoot_SelfProtect(void);

// 发射机构设备
shoot_dev_t		shoot_dev = {
	.fric_left = &fric_motor[FRIC_L],
	.fric_right = &fric_motor[FRIC_R],
	.dial_motor = &dial_motor,
	.rc_sensor = &rc_sensor,
};

// 发射机构信息
shoot_info_t 	shoot_info = {
	.remote_mode = RC,
};

shoot_work_info_t  shoot_work_info = {
	.fric_status = WaitCommond_Fric,
	.dial_status = WaitCommond_Dial,
	.launcher_commond = WaitCommond_L,
	.lock_cnt = 0,
};

shoot_conf_t   shoot_conf = {
	.fric_speed = 2000.0f,
	.dial_speed = -1500.0f,
	.lock_angle_check = 1.5f,
	.lock_cnt = 50,
	.F_Lock_Angle = 30.0f,
};

launcher_t launcher = {
	.dev = &shoot_dev,
	.info = &shoot_info,
	.work_info = &shoot_work_info,
	.conf = &shoot_conf,
	.init = Shoot_Init,
	.ctrl = Shoot_Ctrl,
	.self_protect = Shoot_SelfProtect,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Shoot_Init(void)
{
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	fric_motor[FRIC_L].init(&fric_motor[FRIC_L]);
	fric_motor[FRIC_R].init(&fric_motor[FRIC_R]);
	dial_motor.init(&dial_motor);
	launcher.info->rc_work_state = rc_sensor.work_state;
}

void Shoot_GetBaseInfo(void)
{
	launcher.info->measure_left_speed = fric_motor[FRIC_L].info->speed_rpm;
	launcher.info->measure_right_speed = fric_motor[FRIC_R].info->speed_rpm;
	launcher.info->measure_dial_speed = dial_motor.info->speed_rpm;
	launcher.info->measure_dial_angle = dial_motor.info->angle;
	
}

void Shoot_GetRcState(void)
{
	if ((launcher.info->rc_work_state == DEV_OFFLINE)
		&& rc_sensor.work_state == DEV_ONLINE)
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		if (launcher.info->init_s2 == rc_sensor.info->s2)
		{
			launcher.work_info->launcher_commond = WaitCommond_L;
		}
		else
		{
			launcher.info->init_s2 = 0;
			if (gimbal.info->gimbal_mode == gim_gyro)
			{
				if (rc_sensor.info->s2 == 3)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				else if (rc_sensor.info->s2 == 2)
				{
					launcher.work_info->launcher_commond = Fric_Toggle;
				}
				else if (rc_sensor.info->s2 == 1)
				{
					launcher.work_info->launcher_commond = Keep_Shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
						//launcher.info->target_dial_angle = 45.0f + launcher.info->measure_dial_angle
						arm_offset_f32(&launcher.info->measure_dial_angle, -45.0f, &launcher.info->target_dial_angle, 1);
					}
				}
			}
			else if (gimbal.info->gimbal_mode == gim_machine)
			{
				if (rc_sensor.info->s2 == 3)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				else if (rc_sensor.info->s2 == 2)
				{
					launcher.work_info->launcher_commond = Magz_Open;
				}
				else if (rc_sensor.info->s2 == 1)
				{
					launcher.work_info->launcher_commond = Single_Shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
						//launcher.info->target_dial_angle = 45.0f + launcher.info->measure_dial_angle
						arm_offset_f32(&launcher.info->measure_dial_angle, -45.0f, &launcher.info->target_dial_angle, 1);
					}
				}
			}
		}
	}
	else 
	{
		launcher.work_info->launcher_commond = WaitCommond_L;
		launcher.work_info->fric_status = WaitCommond_Fric;
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
}

void Shoot_GetCtrlInfo(void)
{
	if (launcher.info->remote_mode == RC)
	{
		Shoot_GetRcState();
	}
//	else if (launcher.info->remote_mode == KEY)
//	{
//		
//	}
}

// 摩擦轮跳变检测
void Fric_StepCheck(void)
{
	if ((launcher.work_info->launcher_commond == Fric_Toggle)
		&& (launcher.info->last_s2 != rc_sensor.info->s2))
	{
		if (launcher.work_info->fric_status == WaitCommond_Fric)
		{
			launcher.work_info->fric_status = On_Fric;
		}
		else if (launcher.work_info->fric_status == On_Fric)
		{
			launcher.work_info->fric_status = Off_Fric;
		}
		else if (launcher.work_info->fric_status == Off_Fric)
		{
			launcher.work_info->fric_status = On_Fric;
		}
	}
}

void Dial_StatusCheck(void)
{
	if (launcher.work_info->dial_status == Reload_Dial)
	{
		if (launcher.info->measure_dial_angle < launcher.info->target_dial_angle + launcher.conf->lock_angle_check)
		{
			if (launcher.work_info->launcher_commond == Keep_Shoot)
			{
				//launcher.info->target_dial_angle = -45.0f + launcher.info->measure_dial_angle
				arm_offset_f32(&launcher.info->measure_dial_angle, -45.0f, &launcher.info->target_dial_angle, 1);
			}
			else if (launcher.work_info->launcher_commond == Single_Shoot)
			{
				launcher.work_info->dial_status = WaitCommond_Dial;
			}
		}
		else 
		{
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = F_Lock_Dial;
					//launcher.info->target_dial_angle = launcher.conf->F_Lock_Angle + launcher.info->measure_dial_angle
					arm_add_f32(&launcher.info->measure_dial_angle, &launcher.conf->F_Lock_Angle, &launcher.info->target_dial_angle, 1);
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else if (launcher.work_info->dial_status == F_Lock_Dial)
	{
		if (launcher.info->measure_dial_angle > launcher.info->target_dial_angle - launcher.conf->lock_angle_check)
		{
			//launcher.info->target_dial_angle = -45.0f + launcher.info->measure_dial_angle
			arm_offset_f32(&launcher.info->measure_dial_angle, -45.0f, &launcher.info->target_dial_angle, 1);
			launcher.work_info->dial_status = Reload_Dial;
		}
		else 
		{
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = Reload_Dial;
					//launcher.info->target_dial_angle = -45.0f + launcher.info->measure_dial_angle
					arm_offset_f32(&launcher.info->measure_dial_angle, -45.0f, &launcher.info->target_dial_angle, 1);
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
}

void Get_LauncherStatus(void)
{
	Fric_StepCheck();
	
	Dial_StatusCheck();
	
}

void Shoot_GetInfo(void)
{
	Shoot_GetBaseInfo();
	
	Shoot_GetCtrlInfo();
	
	Get_LauncherStatus();
}

void Fric_Ctrl(void)
{
	if (launcher.work_info->fric_status == On_Fric)
	{
		launcher.info->target_left_speed = -launcher.conf->fric_speed;
		launcher.info->target_right_speed = launcher.conf->fric_speed;
		
		fric_motor[FRIC_L].pid->speed_out = PID_Plc_Calc(&fric_motor[FRIC_L].pid->speed, 
																										 launcher.info->measure_left_speed, 
																										 launcher.info->target_left_speed);
		fric_motor[FRIC_R].pid->speed_out = PID_Plc_Calc(&fric_motor[FRIC_R].pid->speed, 
																										 launcher.info->measure_right_speed, 
																										 launcher.info->target_right_speed);
	}
	else if (launcher.work_info->fric_status == Off_Fric)
	{
		launcher.info->target_left_speed = 0.0f;
		launcher.info->target_right_speed = 0.0f;
		
		fric_motor[FRIC_L].pid->speed_out = PID_Plc_Calc(&fric_motor[FRIC_L].pid->speed, 
																										 launcher.info->measure_left_speed, 
																										 launcher.info->target_left_speed);
		fric_motor[FRIC_R].pid->speed_out = PID_Plc_Calc(&fric_motor[FRIC_R].pid->speed, 
																										 launcher.info->measure_right_speed, 
																										 launcher.info->target_right_speed);
	}
	shoot_out[0] = (int16_t)fric_motor[FRIC_L].pid->speed_out;
	shoot_out[1] = (int16_t)fric_motor[FRIC_R].pid->speed_out;
}

void Dial_Ctrl(void)
{
	if (launcher.work_info->dial_status != WaitCommond_Dial)
	{
		// 过零点处理
		if (launcher.info->target_dial_angle < -360.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		else if (launcher.info->target_dial_angle > 360.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		
		if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle > 180.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		else if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle < -180.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		
		dial_motor.pid->angle_out = PID_Plc_Calc(&dial_motor.pid->angle, 
																						 launcher.info->measure_dial_angle, 
																						 launcher.info->target_dial_angle);
		dial_motor.pid->speed_out = PID_Plc_Calc(&dial_motor.pid->speed, 
																						 launcher.info->measure_dial_speed, 
																						 dial_motor.pid->angle_out);
	}
	else 
	{
		dial_motor.pid->angle_out = PID_Plc_Calc(&dial_motor.pid->angle, 
																						 launcher.info->measure_dial_angle, 
																						 launcher.info->measure_dial_angle);
		dial_motor.pid->speed_out = PID_Plc_Calc(&dial_motor.pid->speed, 
																						 launcher.info->measure_dial_speed, 
																						 dial_motor.pid->angle_out);
	}
	shoot_out[2] = (int16_t)dial_motor.pid->speed_out;
}

void Launcher_Ctrl(void)
{
	Fric_Ctrl();
	Dial_Ctrl();
}

void Launcher_SendOut(void)
{
	// 摩擦轮电机离线保护
	if (fric_motor[FRIC_L].work_state == DEV_ONLINE)
	{
		can2_tx_buf[0] = (shoot_out[fric_motor[FRIC_L].driver->rx_id - 0x201] >> 8) & 0xFF;
		can2_tx_buf[1] = shoot_out[fric_motor[FRIC_L].driver->rx_id - 0x201] & 0xFF;
	}
	else
	{
		can2_tx_buf[0] = 0;
		can2_tx_buf[1] = 0;
	}
	
	if (fric_motor[FRIC_R].work_state == DEV_ONLINE)
	{
		can2_tx_buf[2] = (shoot_out[fric_motor[FRIC_R].driver->rx_id - 0x201] >> 8) & 0xFF;
		can2_tx_buf[3] = shoot_out[fric_motor[FRIC_R].driver->rx_id - 0x201] & 0xFF;
	}
	else
	{
		can2_tx_buf[2] = 0;
		can2_tx_buf[3] = 0;
	}
	
	// 拨盘电机离线保护
	if (dial_motor.work_state == DEV_ONLINE)
	{
		can2_tx_buf[4] = (shoot_out[dial_motor.driver->rx_id - 0x201] >> 8) & 0xFF;
		can2_tx_buf[5] = shoot_out[dial_motor.driver->rx_id - 0x201] & 0xFF;
	}
	else
	{
		can2_tx_buf[4] = 0;
		can2_tx_buf[5] = 0;
	}
	
	launcher.info->rc_work_state = rc_sensor.work_state;
	launcher.info->last_s2 = rc_sensor.info->s2;
}

void Shoot_Ctrl(void)
{
	Shoot_GetInfo();
	
	Launcher_Ctrl();
	
	Launcher_SendOut();
}

// 遥控器失联保护
void Shoot_Stop(void)
{
	fric_motor[FRIC_L].pid->speed_out = 0.0f;
	fric_motor[FRIC_R].pid->speed_out = 0.0f;
	dial_motor.pid->speed_out = 0.0f;
	shoot_out[0] = (int16_t)fric_motor[FRIC_L].pid->speed_out;
	shoot_out[1] = (int16_t)fric_motor[FRIC_R].pid->speed_out;
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
