#include "System_task.h"

system_t sys = {
	.remote_mode = RC,
	.state = SYS_STATE_RCLOST,
	.mode = SYS_MODE_NORMAL,
};

void Start_System_task(void const * argument)
{
	for(;;)
	{
		portENTER_CRITICAL();
		
		// ����ң����Ϣ
		//rc_update_info();
		
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
			/* ң�ش��� */
//			else if(rc_sensor.errno == DEV_DATA_ERR) {
//				sys.state = SYS_STATE_RCERR;
//				//reset CPU
//				__set_FAULTMASK(1);
//				NVIC_SystemReset();
//			} else {
//				sys.state = SYS_STATE_WRONG;
//				//reset CPU
//				__set_FAULTMASK(1);
//				NVIC_SystemReset();
//			}
		}
		
		portEXIT_CRITICAL();
		
		osDelay(2);
	}
}
