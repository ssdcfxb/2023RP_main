#include "Ctrl_task.h"

void Start_Ctrl_task(void const * argument)
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
		
		CAN_Send_All();
		osDelay(1);
	}
}
