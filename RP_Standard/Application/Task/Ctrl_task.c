#include "Ctrl_task.h"

void Start_Ctrl_task(void const * argument)
{
	chassis.init();
	gimbal.init();
	launcher.init();
	for(;;)
	{
		if(sys.state == SYS_STATE_NORMAL) 
		{
			chassis.ctrl();
			gimbal.ctrl();
			launcher.ctrl();
		} 
		else 
		{
			chassis.self_protect();
			gimbal.self_protect();
			launcher.self_protect();
		}
		
		vision_send_data();
		CAN_Send_All();
		osDelay(1);
	}
}
