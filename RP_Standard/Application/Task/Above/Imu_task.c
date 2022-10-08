#include "Imu_task.h"

int16_t Filter_out = 0;

void Start_IMU_task(void const * argument)
{
	
	for(;;)
	{
		taskENTER_CRITICAL();
		// 更新陀螺仪数据
		imu_sensor.update(&imu_sensor);
		taskEXIT_CRITICAL();
		osDelay(1);
	}
	
}


