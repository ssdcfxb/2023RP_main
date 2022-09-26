#include "Imu_task.h"

int16_t accx, accy, accz, gyrox, gyroy, gyroz;
int16_t Filter_out = 0;

void Start_imu_task(void const * argument)
{
	
	for(;;)
	{
		LED_GREEN_ON();
		BMI_Get_RawData(&gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
		Filter_out = 0.1f * accx + 0.9f * Filter_out;
		osDelay(1);
	}
	
}


