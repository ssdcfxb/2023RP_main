#include "motor_3508.h"

extern void CAN_Tx_cmd(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data_1, //临时函数
								       int16_t data_2, int16_t data_3, int16_t data_4);

static void Motor_Init(chassis_motor_t *motor);
static void UpdateMotorData(chassis_motor_t *motor, uint8_t* data);
static void Check_Motor_Data(chassis_motor_t *motor);
static void Chassis_Motor_Heart_Beat(chassis_motor_t *motor);

drv_can_t motor_driver[] = {
	[CHAS_LF] = {
	  .id = DRV_CAN1,
	  .rx_id = RM3508_CAN_ID_201,
	
		.add_msg = CAN_AddMsg,
		.add_byte = CAN_AddByte,
		.add_halfword = CAN_AddHalfWord,
		.add_word = CAN_AddWord,
		.manual_tx = CAN_ManualTx,
	
	  .can_tx_cmd = CAN_Tx_cmd,//临时函数
	},
	[CHAS_RF] = {
	  .id = DRV_CAN1,
	  .rx_id = RM3508_CAN_ID_202,
	
		.add_msg = CAN_AddMsg,
		.add_byte = CAN_AddByte,
		.add_halfword = CAN_AddHalfWord,
		.add_word = CAN_AddWord,
		.manual_tx = CAN_ManualTx,
	},
	[CHAS_LB] = {
	  .id = DRV_CAN1,
	  .rx_id = RM3508_CAN_ID_203,
	
		.add_msg = CAN_AddMsg,
		.add_byte = CAN_AddByte,
		.add_halfword = CAN_AddHalfWord,
		.add_word = CAN_AddWord,
		.manual_tx = CAN_ManualTx,
	},
	[CHAS_RB] = {
	  .id = DRV_CAN1,
	  .rx_id = RM3508_CAN_ID_204,
	
		.add_msg = CAN_AddMsg,
		.add_byte = CAN_AddByte,
		.add_halfword = CAN_AddHalfWord,
		.add_word = CAN_AddWord,
		.manual_tx = CAN_ManualTx,
	},
};

chassis_motor_info_t motor_info[] = {
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

chassis_motor_t chassis_motor[] = {
	[CHAS_LF] = {
    .info = &motor_info[CHAS_LF],
	  .driver = &motor_driver[CHAS_LF],
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
    .init = Motor_Init,
	  .update = UpdateMotorData,
	  .check = Check_Motor_Data,
	  .heart_beat = Chassis_Motor_Heart_Beat,
	  .work_state = DEV_OFFLINE,
	  .id = DEV_ID_CHASSIS_RB,
	},
};

static void Motor_Init(chassis_motor_t *motor)
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
		
		PID_Init(&motor->hpid_speed);
		
		PID_Init(&motor->hpid_angle);
		
}

static void UpdateMotorData(chassis_motor_t *motor, uint8_t *data)
{
	  if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}
		motor->info->ecd = (uint16_t)((data[0] << 8) | data[1]);
		motor->info->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
		motor->info->given_current = (int16_t)((data[4] << 8) | data[5]);
		
}

static void Check_Motor_Data(chassis_motor_t *motor)
{
		if (motor->info == NULL || motor == NULL)
		{
			motor->errno = DEV_INIT_ERR;
			return;
		}

		chassis_motor_info_t *motor_info = motor->info;

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
		motor->info->angle = motor->info->total_ecd * M3508_ECD_TO_ANGLE;
		
		motor->info->offline_cnt = 0;
}

static void Chassis_Motor_Heart_Beat(chassis_motor_t *motor)
{
	if (motor->info == NULL || motor == NULL)
	{
		motor->errno = DEV_INIT_ERR;
		return;
	}
	
	chassis_motor_info_t *motor_info = motor->info;
	
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



//static int16_t delta_ecd_calc(uint16_t ecd, uint16_t last_ecd)
//{
//    int16_t delta_ecd = ecd - last_ecd;
//    if (delta_ecd > HALF_ECD_RANGE)
//    {
//        delta_ecd -= ECD_RANGE;
//    }
//    else if (delta_ecd < -HALF_ECD_RANGE)
//    {
//        delta_ecd += ECD_RANGE;
//    }

//    return delta_ecd ;
//}
