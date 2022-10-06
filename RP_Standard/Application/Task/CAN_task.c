#include "CAN_task.h"

//float angle_set = 0.0f;
//float speed_out = 0.0f;
//float angle_out = 0.0f;

uint8_t tx_data[16];

void Start_CAN_task(void const * argument)
{
	CAN_Filter_Init();
	for(;;)
	{
		memcpy(tx_data, can1_tx_buf, 16);
		
		if (rc_sensor.work_state == DEV_OFFLINE && flag.chassis_flag.stop_ok == 1)
		{
			memset(tx_data, 0, 16);
		}
		
		CAN_SendData(&hcan1, 0x200, tx_data);
		CAN_SendData(&hcan1, 0x1FF, &tx_data[8]);
		
	  memset(tx_data, 0, 16);
	
		osDelay(1);
	}
	
}
