#include "Chassis_task.h"

void Start_Chassis_task(void const * argument)
{
	chassis.init();
	gimbal.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) 
		{
			chassis.ctrl();
			gimbal.ctrl();
		} 
		else 
		{
			chassis.self_protect();
			gimbal.self_protect();
		}
		
		osDelay(2);
	}
}
