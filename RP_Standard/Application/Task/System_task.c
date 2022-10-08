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
		// ����ң����Ϣ
		rc_update_info();
		
		/* ң������ */
		if(rc_sensor.work_state == DEV_OFFLINE) 
		{
			sys.state = SYS_STATE_RCLOST;
			RC_ResetData(&rc_sensor);
		} 
		/* ң������ */
		else if(rc_sensor.work_state == DEV_ONLINE)
		{
			/* ң������ */
			if(rc_sensor.errno == NONE_ERR) 
			{
				/* ʧ���ָ� */
				if(sys.state == SYS_STATE_RCLOST) 
				{
					// ���ڴ˴�ͬ����̨��λ��־λ					
					// ϵͳ������λ
					sys.remote_mode = RC;
					sys.state = SYS_STATE_NORMAL;
					sys.mode = SYS_MODE_NORMAL;
				}
				
				// ���ڴ˴��ȴ���̨��λ��������л�״̬
//				system_state_machine();
			}
		}
		
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
