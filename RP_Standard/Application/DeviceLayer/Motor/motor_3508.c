#include "motor_3508.h"


static void Motor_Init(motor_3508_t *motor);
static void UpdateMotorData(motor_3508_t *motor, uint8_t* data);
static void Check_Motor_Data(motor_3508_t *motor);
static void Chassis_Motor_Heart_Beat(motor_3508_t *motor);

drv_can_t motor_driver[] = {
	[CHAS_LF] = {
	  .hcan = &hcan1,
	  .rx_id = RM3508_CAN_ID_201,
	},
	[CHAS_RF] = {
	  .hcan = &hcan1,
	  .rx_id = RM3508_CAN_ID_202,
	},
	[CHAS_LB] = {
	  .hcan = &hcan1,
	  .rx_id = RM3508_CAN_ID_203,
	},
	[CHAS_RB] = {
	  .hcan = &hcan1,
	  .rx_id = RM3508_CAN_ID_204,
	},
};

motor_3508_info_t motor_info[] = {
	{
    .offline_max_cnt = 50,
	},
	{
    .offline_max_cnt = 50,
	},
	{
    .offline_max_cnt = 50,
	},
	{
    .offline_max_cnt = 50,
	},
};

// µç»úPID(Plc-Plc)
pid_t pid[] = {
	[CHAS_LF] = {
		.speed.Kp = M3508_SP_KP,
		.speed.Ki = M3508_SP_KI,
		.speed.Kd = M3508_SP_KD,
		.speed.max_out = CHAS_SP_MAX_OUT,
		.speed.max_integral = CHAS_SP_MAX_INTEGRAL,
		.speed.max_iout = CHAS_SP_MAX_I_OUT,
		.angle.Kp = M3508_AG_KP,
		.angle.Ki = M3508_AG_KI,
		.angle.Kd = M3508_AG_KD,
		.angle.max_out = CHAS_AG_MAX_OUT,
		.angle.max_integral = CHAS_AG_MAX_INTEGRAL,
		.angle.max_iout = CHAS_AG_MAX_I_OUT,
	},
	[CHAS_RF] = {
		.speed.Kp = M3508_SP_KP,
		.speed.Ki = M3508_SP_KI,
		.speed.Kd = M3508_SP_KD,
		.speed.max_out = CHAS_SP_MAX_OUT,
		.speed.max_integral = CHAS_SP_MAX_INTEGRAL,
		.speed.max_iout = CHAS_SP_MAX_I_OUT,
		.angle.Kp = M3508_AG_KP,
		.angle.Ki = M3508_AG_KI,
		.angle.Kd = M3508_AG_KD,
		.angle.max_out = CHAS_AG_MAX_OUT,
		.angle.max_integral = CHAS_AG_MAX_INTEGRAL,
		.angle.max_iout = CHAS_AG_MAX_I_OUT,
	},
	[CHAS_LB] = {
		.speed.Kp = M3508_SP_KP,
		.speed.Ki = M3508_SP_KI,
		.speed.Kd = M3508_SP_KD,
		.speed.max_out = CHAS_SP_MAX_OUT,
		.speed.max_integral = CHAS_SP_MAX_INTEGRAL,
		.speed.max_iout = CHAS_SP_MAX_I_OUT,
		.angle.Kp = M3508_AG_KP,
		.angle.Ki = M3508_AG_KI,
		.angle.Kd = M3508_AG_KD,
		.angle.max_out = CHAS_AG_MAX_OUT,
		.angle.max_integral = CHAS_AG_MAX_INTEGRAL,
		.angle.max_iout = CHAS_AG_MAX_I_OUT,
	},
	[CHAS_RB] = {
		.speed.Kp = M3508_SP_KP,
		.speed.Ki = M3508_SP_KI,
		.speed.Kd = M3508_SP_KD,
		.speed.max_out = CHAS_SP_MAX_OUT,
		.speed.max_integral = CHAS_SP_MAX_INTEGRAL,
		.speed.max_iout = CHAS_SP_MAX_I_OUT,
		.angle.Kp = M3508_AG_KP,
		.angle.Ki = M3508_AG_KI,
		.angle.Kd = M3508_AG_KD,
		.angle.max_out = CHAS_AG_MAX_OUT,
		.angle.max_integral = CHAS_AG_MAX_INTEGRAL,
		.angle.max_iout = CHAS_AG_MAX_I_OUT,
	},
};

motor_3508_t chassis_motor[] = {
	[CHAS_LF] = {
    .info = &motor_info[CHAS_LF],
	  .driver = &motor_driver[CHAS_LF],
		.pid = &pid[CHAS_LF],
    .init = Motor_Init,
	  .update = UpdateMotorData,
	  .check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
	  .work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_LF,
	},
	[CHAS_RF] = {
    .info = &motor_info[CHAS_RF],
	  .driver = &motor_driver[CHAS_RF],
		.pid = &pid[CHAS_RF],
    .init = Motor_Init,
	  .update = UpdateMotorData,
	  .check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
	  .work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_RF,
	},
	[CHAS_LB] = {
    .info = &motor_info[CHAS_LB],
	  .driver = &motor_driver[CHAS_LB],
		.pid = &pid[CHAS_LB],
    .init = Motor_Init,
	  .update = UpdateMotorData,
	  .check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
	  .work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_LB,
	},
	[CHAS_RB] = {
    .info = &motor_info[CHAS_RB],
	  .driver = &motor_driver[CHAS_RB],
		.pid = &pid[CHAS_RB],
    .init = Motor_Init,
	  .update = UpdateMotorData,
	  .check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
	  .work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_RB,
	},
};

drv_can_t fric_motor_driver[] = {
	[FRIC_L] = {
	  .hcan = &hcan2,
	  .rx_id = RM3508_CAN_ID_201,
	},
	[FRIC_R] = {
	  .hcan = &hcan2,
	  .rx_id = RM3508_CAN_ID_202,
	},
};

motor_3508_info_t fric_motor_info[] = {
	{
    .offline_max_cnt = 50,
	},
	{
    .offline_max_cnt = 50,
	},
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void Motor_Init(motor_3508_t *motor)
{
		if (motor->info == NULL || motor == NULL || motor->driver == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}
		motor->info->angle = 0;
		motor->info->total_ecd = 0;
		motor->info->offline_cnt = 51;
		motor->errno = NONE_ERR;
		motor->work_state = DEV_OFFLINE;
		
		PID_Init(&motor->pid->speed);
		PID_Init(&motor->pid->angle);
}

static void UpdateMotorData(motor_3508_t *motor, uint8_t *data)
{
	  if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}
		motor->info->ecd = (uint16_t)((data[0] << 8) | data[1]);
		motor->info->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
		motor->info->given_current = (int16_t)((data[4] << 8) | data[5]);
		motor->info->temperature = (int8_t)data[6];
}

static void Check_Motor_Data(motor_3508_t *motor)
{
		if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}

		motor_3508_info_t *motor_info = motor->info;

		motor_info->delta_ecd = motor_info->ecd - motor_info->last_ecd;
		if (motor_info->delta_ecd > HALF_ECD_RANGE)
		{
				motor_info->delta_ecd -= ECD_RANGE;
		}
		else if (motor_info->delta_ecd < -HALF_ECD_RANGE)
		{
				motor_info->delta_ecd += ECD_RANGE;
		}
	
	  motor->info->last_ecd = motor->info->ecd;
		motor->info->total_ecd += motor->info->delta_ecd;
		motor->info->angle = motor->info->ecd * M3508_ECD_TO_ANGLE;
		motor->info->total_angle = motor->info->total_ecd * M3508_ECD_TO_ANGLE;
		
		motor->info->offline_cnt = 0;
}

static void Chassis_Motor_Heart_Beat(motor_3508_t *motor)
{
	if (motor->info == NULL || motor == NULL)
	{
		motor->errno = DEV_INIT_ERR;
		return;
	}
	
	motor_3508_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) 
	{
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else 
	{
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

