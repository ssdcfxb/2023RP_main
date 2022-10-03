#include "CAN_task.h"

//float angle_set = 0.0f;
//float speed_out = 0.0f;
//float angle_out = 0.0f;

void Start_CAN_task(void const * argument)
{
	CAN_Filter_Init();
	chassis_motor[CHAS_LF].init(&chassis_motor[CHAS_LF]);
	motor_6020.init(&motor_6020);
	
	for(;;)
	{
//		if ((chassis_motor[CHAS_LF].errno != NONE_ERR) || (chassis_motor[CHAS_LF].work_state == DEV_OFFLINE))
//		{
//			CAN_Tx_Cmd(chassis_motor[CHAS_LF].driver->hcan, RM3508_GetTxId(chassis_motor[CHAS_LF].driver), 0, 0, 0, 0);
//		}
////		if ((motor_6020.errno != NONE_ERR) || (motor_6020.work_state == DEV_OFFLINE))
////		{
////			CAN_Tx_Cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), 0, 0, 0, 0);
////		}
//		else
//		{
//			LED_RED_OFF();
////			chassis_motor[CHAS_LF].pid->angle_out = PID_Plc_Calc(&chassis_motor[CHAS_LF].pid->angle, chassis_motor[CHAS_LF].info->angle, angle_set);
////			angle_out = chassis_motor[CHAS_LF].pid->angle_out;
////			chassis_motor[CHAS_LF].pid->speed_out = PID_Inc_Calc(&chassis_motor[CHAS_LF].pid->speed, chassis_motor[CHAS_LF].info->speed_rpm, angle_out);
////			speed_out = chassis_motor[CHAS_LF].pid->speed_out;
////			CAN_Tx_Cmd(chassis_motor[CHAS_LF].driver->hcan, RM3508_GetTxId(chassis_motor[CHAS_LF].driver), (int16_t)speed_out, 0, 0, 0);
//			
////			CAN_Tx_Cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), speed, 0, 0, 0);
//		}
		
		CAN_SendData(&hcan1, 0x200, can1_tx_buf);
		
		osDelay(1);
	}
	
}
