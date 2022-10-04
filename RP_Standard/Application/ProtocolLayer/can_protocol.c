/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"


void CAN1_Get_Data(uint32_t identifier, uint8_t *data)
{//����CAN1���յ�����
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
{//����CAN2���յ�����
	switch (identifier)
	{
		
	}
}


