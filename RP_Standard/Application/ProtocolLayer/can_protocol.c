/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

uint8_t can1_tx_data[16];
uint8_t can2_tx_data[16];

void CAN_Send_All(void)
{
	memcpy(can1_tx_data, can1_tx_buf, 16);
	memcpy(can2_tx_data, can2_tx_buf, 16);
		
	if (rc_sensor.work_state == DEV_OFFLINE && flag.chassis_flag.stop_ok == 1)
	{
		memset(can1_tx_data, 0, 16);
		memset(can2_tx_data, 0, 16);
	}
	
	CAN_SendData(&hcan1, 0x200, can1_tx_data);
	CAN_SendData(&hcan1, 0x1FF, &can1_tx_data[8]);
	CAN_SendData(&hcan2, 0x200, can2_tx_data);
	
	memset(can1_tx_data, 0, 16);
	memset(can2_tx_data, 0, 16);
}

void CAN1_Get_Data(uint32_t identifier, uint8_t *data)
{//处理CAN1接收的数据
	switch (identifier)
	{
		case RM3508_CAN_ID_201:
		{
			chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], data);
	    chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
		  break;
		}
		case RM3508_CAN_ID_202:
		{
			chassis_motor[CHAS_RF].update(&chassis_motor[CHAS_RF], data);
	    chassis_motor[CHAS_RF].check(&chassis_motor[CHAS_RF]);
		  break;
		}
		case RM3508_CAN_ID_203:
		{
			chassis_motor[CHAS_LB].update(&chassis_motor[CHAS_LB], data);
	    chassis_motor[CHAS_LB].check(&chassis_motor[CHAS_LB]);
		  break;
		}
		case RM3508_CAN_ID_204:
		{
			chassis_motor[CHAS_RB].update(&chassis_motor[CHAS_RB], data);
	    chassis_motor[CHAS_RB].check(&chassis_motor[CHAS_RB]);
		  break;
		}
		case GM6020_CAN_ID_205:
		{
			yaw_motor.update(&yaw_motor, data);
	    yaw_motor.check(&yaw_motor);
		  break;
		}
		case GM6020_CAN_ID_206:
		{
			pitch_motor.update(&pitch_motor, data);
			pitch_motor.check(&pitch_motor);
			break;
		}
		default :
		{
			
		}
	}
}

void CAN2_Get_Data(uint32_t identifier, uint8_t *data)
{//处理CAN2接收的数据
	switch (identifier)
	{
		case RM3508_CAN_ID_201:
		{
			shoot_motor[FRIC_L].update(&chassis_motor[FRIC_L], data);
	    shoot_motor[FRIC_L].check(&chassis_motor[FRIC_L]);
		  break;
		}
		case RM3508_CAN_ID_202:
		{
			shoot_motor[FRIC_R].update(&chassis_motor[FRIC_R], data);
	    shoot_motor[FRIC_R].check(&chassis_motor[FRIC_R]);
		  break;
		}
		case RM2006_CAN_ID_203:
		{
			dial_motor.update(&dial_motor, data);
	    dial_motor.check(&dial_motor);
		  break;
		}
		default :
		{
			
		}
	}
}


