#include "control.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t i = 0;
	
	if (++i == 60000)
	{
		i = 0;
	}
	
	// 100Hz
	if (i % 10 == 1)
	{
		up_send_power_heat();
	}
	if (i % 10 == 2)
	{
		up_send_shoot();
	}
}

