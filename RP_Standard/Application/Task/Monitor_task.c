#include "Monitor_task.h"

void Start_Monitor_task(void const * argument)
{
	for(;;)
	{
		HAL_IWDG_Refresh(&hiwdg);
		motor_data.Heart_Beat(&motor_data);
		osDelay(1);
	}
	
}
