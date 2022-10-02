#include "motor_6020.h"

static void Motor_Init(motor_6020_t *motor);
static void UpdateMotorData(motor_6020_t *motor, uint8_t* data);
static void Check_Motor_Data(motor_6020_t *motor);
static void Chassis_Motor_Heart_Beat(motor_6020_t *motor);

drv_can_t motor_6020_driver = {
    .hcan = &hcan1,
    .rx_id = GM6020_CAN_ID_205,
};

motor_6020_info_t motor_6020_info = {
    .offline_max_cnt = 50,
};

motor_6020_t motor_6020 = {
    .info = &motor_6020_info,
	  .driver = &motor_6020_driver,
    .init = Motor_Init,
  	.update = UpdateMotorData,
  	.check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
  	.work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_LF,
};

static void Motor_Init(motor_6020_t *motor)
{
		if (motor->info == NULL || motor == NULL || motor->driver == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}
		motor->info->angle = 0;
		motor->info->total_ecd = 0;
		motor->info->offline_cnt = 0;
		motor->errno = NONE_ERR;
		motor->work_state = DEV_OFFLINE;
		
}

static void UpdateMotorData(motor_6020_t *motor, uint8_t *data)
{
	  if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}
		motor->info->ecd = GM6020_GetMotorAngle(data);
		motor->info->speed_rpm = GM6020_GetMotorSpeed(data);
		motor->info->given_current = GM6020_GetMotorCurrent(data);
		motor->info->temperature = GM6020_GetMotorTemperature(data);
}

static void Check_Motor_Data(motor_6020_t *motor)
{
		if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}

		motor_6020_info_t *motor_info = motor->info;

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
		motor->info->angle = motor->info->total_ecd * GM6020_ECD_TO_ANGLE;
		
		motor->info->offline_cnt = 0;
}

static void Chassis_Motor_Heart_Beat(motor_6020_t *motor)
{
	if (motor->info == NULL || motor == NULL)
	{
		motor->errno = DEV_INIT_ERR;
		return;
	}
	
	motor_6020_info_t *motor_info = motor->info;
	
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




