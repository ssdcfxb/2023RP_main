#include "Chassis_task.h"

void Start_Chassis_task(void const * argument)
{
	chassis.init();
	gimbal.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) 
		{
			flag.chassis_flag.stop_start = 0;
			flag.chassis_flag.stop_ok = 0;
			chassis.ctrl();
			gimbal.ctrl();
		} 
		else 
		{
			flag.chassis_flag.stop_start = 1;
			chassis.self_protect();
			gimbal.self_protect();
		}
		
		osDelay(2);
	}
}
