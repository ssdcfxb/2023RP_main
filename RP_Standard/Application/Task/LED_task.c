#include "LED_task.h"

uint16_t i = 0;

void LED_Flash(void);

void Start_LED_task(void const * argument)
{
	LED_GREEN_OFF();
	for(;;)
	{
		LED_Flash();
		osDelay(1);
	}
	
}

void LED_Flash(void)
{
	if (++i == 60000)
		{
			i = 0;
		}
		if ((chassis_motor[CHAS_LF].errno != NONE_ERR) || (chassis_motor[CHAS_LF].work_state == DEV_OFFLINE))
		{
			if (i % 2000 == 0)
			{
			  LED_RED_ON();
			}
			if (i % 2000 == 250)
			{
				LED_RED_OFF();
			}
		}
		
		if ((motor_6020.errno != NONE_ERR) || (motor_6020.work_state == DEV_OFFLINE))
		{
			if (i % 2000 == 500)
			{
			  LED_RED_ON();
			}
			if (i % 2000 == 750)
			{
				LED_RED_OFF();
			}
		}
		
		if(rc_sensor.work_state == DEV_OFFLINE)
		{
			if (i % 1000 == 0)
			{
			  LED_BLUE_ON();
			}
			if (i % 1000 == 500)
			{
				LED_BLUE_OFF();
			}
		}
		else if (rc_sensor.work_state == DEV_ONLINE)
		{
			LED_BLUE_OFF();
		}
}

