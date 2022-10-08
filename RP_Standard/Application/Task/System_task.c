#include "System_task.h"

system_t sys = {
	.remote_mode = RC,
	.state = SYS_STATE_RCLOST,
	.mode = SYS_MODE_NORMAL,
};

flag_t flag = {
	.gimbal_flag.reset_start = 1,
	.gimbal_flag.reset_ok = 0,
	.chassis_flag.stop_start = 0,
	.chassis_flag.stop_ok = 0,
};

void rc_update_info(void)
{
	if(sys.state != SYS_STATE_NORMAL) {
			
	}
	else {
		if (rc_sensor.info->s1 == 3 && rc_sensor.info->s2 == 3)
		{
			gimbal.info->yaw_mode = G_Y_follow;
			gimbal.info->pitch_mode = G_P_machine;
		}
		else if (rc_sensor.info->s1 == 3 && rc_sensor.info->s2 == 2)
		{
			gimbal.info->yaw_mode = G_Y_gyro;
			gimbal.info->pitch_mode = G_P_gyro;
		}
		else 
		{
			gimbal.info->yaw_mode = G_Y_keep;
		}
	}
}

void Start_System_task(void const * argument)
{
	for(;;)
	{
		portENTER_CRITICAL();
		// 更新遥控信息
		rc_update_info();
		
		/* 遥控离线 */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 
		/* 遥控在线 */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* 遥控正常 */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* 失联恢复 */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// 可在此处同步云台复位标志位					
					// 系统参数复位
					sys.remote_mode = RC;
					sys.state = SYS_STATE_NORMAL;
					sys.mode = SYS_MODE_NORMAL;
				}
				
				// 可在此处等待云台复位后才允许切换状态
//				system_state_machine();
			}
		}
		
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
