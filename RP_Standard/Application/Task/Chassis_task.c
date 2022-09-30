#include "Chassis_task.h"

void StartControlTask(void const * argument)
{
//	chassis.init();
	for(;;)
	{
//		if(sys.state == SYS_STATE_NORMAL) {
//			chassis.ctrl();
//		} else {
//			chassis.self_protect();
//		}
		
		osDelay(2);
	}
}
