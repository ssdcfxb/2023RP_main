/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

uint8_t can1_tx_data[16];
uint8_t can2_tx_data[16];

void CAN_Send_All(void)
{
}

void CAN1_Get_Data(uint32_t identifier, uint8_t *data)
{//处理CAN1接收的数据
	switch (identifier)
	{
//		case RM3508_CAN_ID_201:
//		{
//			chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], data);
//	    chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
//		  break;
//		}
		default :
		{
			
		}
	}
}

void CAN2_Get_Data(uint32_t identifier, uint8_t *data)
{//处理CAN2接收的数据
	switch (identifier)
	{
//		case RM3508_CAN_ID_201:
//		{
//			fric_motor[FRIC_L].update(&fric_motor[FRIC_L], data);
//	    fric_motor[FRIC_L].check(&fric_motor[FRIC_L]);
//		  break;
//		}
		default :
		{
			
		}
	}
}


