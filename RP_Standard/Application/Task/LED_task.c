#include "LED_task.h"

uint16_t i = 0;

void LED_Flash(void);
void Chassis_Motor_Flash(void);
void Gimbal_Motor_Flash(void);
void RC_Flash(void);

void Start_LED_task(void const * argument)
{
	LED_GREEN_OFF();
	LED_RED_OFF();
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
	Chassis_Motor_Flash();
	Gimbal_Motor_Flash();
	RC_Flash();
}

void Chassis_Motor_Flash(void)
{
	for (int16_t j = 0; j < CHAS_MOTOR_CNT; j++)
	{
		if ((chassis_motor[j].errno != NONE_ERR) || (chassis_motor[j].work_state == DEV_OFFLINE))
		{
			if (i % 2000 > j * 250 && i % 2000 <= (125 + j * 250))
			{
				LED_RED_ON();
			}
			else if (i % 2000 > (125 + j * 250) && i % 2000 <= (250 + j * 250))
			{
				LED_RED_OFF();
			}
		}
		else 
		{
			LED_RED_OFF();
		}
	}
}

void Gimbal_Motor_Flash(void)
{
	if (yaw_motor.work_state == DEV_OFFLINE)
	{
		if (i % 1000 == 0)
		{
			LED_GREEN_ON();
		}
		else if (i % 1000 == 125)
		{
			LED_GREEN_OFF();
		}
	}
	else 
	{
		LED_GREEN_OFF();
	}
	
	if (pitch_motor.work_state == DEV_OFFLINE)
	{
		if (i % 1000 == 250)
		{
			LED_GREEN_ON();
		}
		else if (i % 1000 == 375)
		{
			LED_GREEN_OFF();
		}
	}
	else
	{
		LED_GREEN_OFF();
	}
}

void RC_Flash(void)
{
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

