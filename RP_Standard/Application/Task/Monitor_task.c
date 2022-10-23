#include "Monitor_task.h"

void Start_Monitor_task(void const * argument)
{
	for(;;)
	{
		HAL_IWDG_Refresh(&hiwdg);
		imu_sensor.heart_beat(&imu_sensor);
		taskENTER_CRITICAL();
		// ��������������
		imu_sensor.update(&imu_sensor);
		taskEXIT_CRITICAL();
		rc_sensor.heart_beat(&rc_sensor);
		for(uint8_t i = 0; i < CHAS_MOTOR_CNT; i++) 
		{
			chassis_motor[i].heart_beat(&chassis_motor[i]);
		}
		fric_motor[0].heart_beat(&fric_motor[0]);
		fric_motor[1].heart_beat(&fric_motor[1]);
		dial_motor.heart_beat(&dial_motor);
		yaw_motor.heart_beat(&yaw_motor);
		pitch_motor.heart_beat(&pitch_motor);
		vision_sensor.heart_beat(&vision_sensor);
		osDelay(1);
	}
	
}

