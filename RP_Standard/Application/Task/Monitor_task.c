#include "Monitor_task.h"

void Start_Monitor_task(void const * argument)
{
	for(;;)
	{
		HAL_IWDG_Refresh(&hiwdg);
		imu_sensor.heart_beat(&imu_sensor);
		rc_sensor.heart_beat(&rc_sensor);
		chassis_motor[CHAS_LF].heart_beat(&chassis_motor[CHAS_LF]);
		motor_6020.heart_beat(&motor_6020);
		osDelay(1);
	}
	
}

