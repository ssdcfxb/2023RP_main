#include "chassis.h"

// �ӽṹ�壨���ڱ�д��鿴��
drv_can_t				  *chas_drv[CHAS_MOTOR_CNT];
motor_3508_t			*chas_motor[CHAS_MOTOR_CNT];
motor_3508_info_t	*chas_motor_info[CHAS_MOTOR_CNT];
int16_t            out[CHAS_MOTOR_CNT];

int16_t speed_out;
float target;

void Chassis_Init(void);
void Chassis_Ctrl(void);
void Chassis_Test(void);
void Chassis_SelfProtect(void);

// ����ģ�鴫����
chassis_dev_t		chas_dev = {
	.chas_motor[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_motor[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_motor[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_motor[CHAS_RB] = &chassis_motor[CHAS_RB],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// ����ģ����Ϣ
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.local_mode = CHASSIS_MODE_NORMAL,
};

chassis_t chassis = {
	.dev = &chas_dev,
	.info = &chas_info,
	.init = Chassis_Init,
	.test = Chassis_Test,
	.ctrl = Chassis_Ctrl,
	.self_protect = Chassis_SelfProtect,
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ���̵����ʼ��
void Chassis_Motor_Init(void)
{
	chassis_motor[CHAS_LF].init(&chassis_motor[CHAS_LF]);
	chassis_motor[CHAS_RF].init(&chassis_motor[CHAS_RF]);
	chassis_motor[CHAS_LB].init(&chassis_motor[CHAS_LB]);
	chassis_motor[CHAS_RB].init(&chassis_motor[CHAS_RB]);
}

void Chassis_Init(void)
{
	// �����ӽṹ���ʼ��
	chas_motor[CHAS_LF] = chas_dev.chas_motor[CHAS_LF];
	chas_motor[CHAS_RF] = chas_dev.chas_motor[CHAS_RF];
	chas_motor[CHAS_LB] = chas_dev.chas_motor[CHAS_LB];
	chas_motor[CHAS_RB] = chas_dev.chas_motor[CHAS_RB];	
	
	chas_motor_info[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->info;
	chas_motor_info[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->info;
	chas_motor_info[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->info;
	chas_motor_info[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->info;
	
	chas_drv[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->driver;
	chas_drv[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->driver;
	chas_drv[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->driver;
	chas_drv[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->driver;
	
	// ���̵����ʼ��
	Chassis_Motor_Init();

}

void Chassis_GetRcInfo(void)
{
	chassis.info->target_front_speed = (float)rc_sensor.info->ch3 * 8000.0f / 660.0f;
	chassis.info->target_right_speed = (float)rc_sensor.info->ch2 * 8000.0f / 660.0f;
	chassis.info->target_cycle_speed = (float)rc_sensor.info->ch0 * 8000.0f / 660.0f;
}

void Chassis_GetKeyInfo(void)
{
	
}

void Chassis_GetInfo(void)
{
	if(chas_info.remote_mode == RC) {
		Chassis_GetRcInfo();
	}
	else if(chas_info.remote_mode == KEY) {
		Chassis_GetKeyInfo();
	}
}

// ���̵��ж��
static void Chassis_Stop(chassis_dev_t *chas_dev)
{
	for(uint8_t i = 0; i < CHAS_MOTOR_CNT; i++)
	{
		chas_motor[i]->pid->speed_out = 0.0f;
		out[i] = (int16_t)chas_motor[i]->pid->speed_out;
	}
	can1_tx_buf[(chas_drv[CHAS_LF]->rx_id - 0x201) * 2] = (out[CHAS_LF] >> 8) & 0xFF;
	can1_tx_buf[(chas_drv[CHAS_LF]->rx_id - 0x201) * 2 + 1] = (out[CHAS_LF] & 0xFF);
	can1_tx_buf[(chas_drv[CHAS_RF]->rx_id - 0x201) * 2] = (out[CHAS_RF] >> 8) & 0xFF;
	can1_tx_buf[(chas_drv[CHAS_RF]->rx_id - 0x201) * 2 + 1] = (out[CHAS_RF] & 0xFF);
	can1_tx_buf[(chas_drv[CHAS_LB]->rx_id - 0x201) * 2] = (out[CHAS_LB] >> 8) & 0xFF;
	can1_tx_buf[(chas_drv[CHAS_LB]->rx_id - 0x201) * 2 + 1] = (out[CHAS_LB] & 0xFF);
	can1_tx_buf[(chas_drv[CHAS_RB]->rx_id - 0x201) * 2] = (out[CHAS_RB] >> 8) & 0xFF;
	can1_tx_buf[(chas_drv[CHAS_RB]->rx_id - 0x201) * 2 + 1] = (out[CHAS_RB] & 0xFF);
}

void Chassis_SelfProtect(void)
{
	Chassis_Stop(&chas_dev);
//	Chassis_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
	Chassis_GetInfo();
}

// ���̵���ٶȻ�����
static void Chassis_Speed_PidCalc(chassis_dev_t *chas_dev, chassis_motor_cnt_t MOTORx)
{
		chas_motor[MOTORx]->pid->speed_out = PID_Inc_Calc(&chas_motor[MOTORx]->pid->speed, 
	                                                chas_motor[MOTORx]->info->speed_rpm, //measure
                                                  chas_motor[MOTORx]->pid->speed.set); //target
		out[MOTORx] = (int16_t)chas_motor[MOTORx]->pid->speed_out;
}

// ��ӵ��̵����������ͻ���
static void Chassis_SendPidOut(chassis_dev_t *chas_dev)
{
	for(uint8_t i = 0; i < CHAS_MOTOR_CNT; i++) 
	{
		if(chas_motor[i]->work_state == DEV_ONLINE) 
		{
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2] = (out[i] >> 8) & 0xFF;
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2 + 1] = (out[i] & 0xFF);
		} 
		else 
		{
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2] = 0;
			can1_tx_buf[(chas_motor[i]->driver->rx_id - 0x201) * 2 + 1] = 0;
		}
	}    
}

void Chassis_PidCtrl(void)
{
	// ���̵���ٶȻ�����
	Chassis_Speed_PidCalc(&chas_dev, CHAS_LF);
	Chassis_Speed_PidCalc(&chas_dev, CHAS_RF);
	Chassis_Speed_PidCalc(&chas_dev, CHAS_LB);
	Chassis_Speed_PidCalc(&chas_dev, CHAS_RB);
	
	speed_out = out[CHAS_LF];
	
	// ��ӵ��̵����������ͻ���
	Chassis_SendPidOut(&chas_dev);
}

void Chassis_RcCtrl(void)
{
	int16_t front, right, round;
	
	front = chassis.info->target_front_speed;
	right = chassis.info->target_right_speed;
	round = chassis.info->target_cycle_speed;
	
	chas_motor[CHAS_LF]->pid->speed.set = front + right + round;
  chas_motor[CHAS_RF]->pid->speed.set = - front + right + round;
  chas_motor[CHAS_LB]->pid->speed.set = front - right + round;
  chas_motor[CHAS_RB]->pid->speed.set = - front - right + round;
	target = chas_motor[CHAS_LF]->pid->speed.set;
}

void Chassis_Ctrl(void)
{
	/*----��Ϣ����----*/
	Chassis_GetInfo();
	/*----���Ʒ�ʽ�޸�----*/ 
	if(chas_info.remote_mode == RC) {
		Chassis_RcCtrl();
	}
//	else if(chas_info.remote_mode == KEY) {
//		Chassis_KeyCtrl();
//	}
	/*----�������----*/
	Chassis_PidCtrl();
}

void Chassis_Test(void)
{
    
}

