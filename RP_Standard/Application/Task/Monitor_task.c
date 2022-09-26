#include "Monitor_task.h"

void Start_Monitor_task(void const * argument)
{
	for(;;)
	{
		motor_data.Heart_Beat(&motor_data);
		osDelay(1);
	}
	
}

