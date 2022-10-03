#include "Chassis_task.h"

void Start_Chassis_task(void const * argument)
{
	chassis.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) 
		{
			chassis.ctrl();
		} 
		else 
		{
			chassis.self_protect();
		}
		
		osDelay(2);
	}
}
