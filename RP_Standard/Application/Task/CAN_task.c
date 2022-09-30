#include "CAN_task.h"

float angle_set = 0.0f;
float speed_out = 0.0f;
float angle_out = 0.0f;

uint8_t tx_info[8] = {0};

void trans(uint8_t data[8])
{
	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mail_box;
	uint8_t *idx;
	
	idx = (uint8_t *)&info_pack.my_info->height;
	
	data[3] = *idx;
	data[4] = *(idx + 1);
	data[5] = *(idx + 2);
	data[6] = *(idx + 3);
	
	data[0] = (uint8_t)info_pack.my_info->age;
	
	
	tx_header.StdId = 0x123;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mail_box);
}

void reserve(uint8_t data[8])
{
	uint8_t *idx;
	idx = (uint8_t *)&info_pack.get_info->height;
	
	idx[0] = data[3];
	idx[1] = data[4];
	idx[2] = data[5];
	idx[3] = data[6];
	
	info_pack.get_info->age = (char)data[0];
	
}

void Start_CAN_task(void const * argument)
{
	CAN_filter_init();
	motor_data.Init(&motor_data);
	motor_6020.Init(&motor_6020);
	
//	speed_out = 500.0f;
	for(;;)
	{
		if ((motor_data.errno != NONE_ERR) || (motor_data.work_state == DEV_OFFLINE))
		{
			motor_data.driver->can_tx_cmd(&hcan1, RM3508_GetTxId(motor_data.driver), 0, 0, 0, 0);
			LED_RED_ON();
		}
//		if ((motor_6020.errno != NONE_ERR) || (motor_6020.work_state == DEV_OFFLINE))
//		{
//			motor_data.driver->can_tx_cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), 0, 0, 0, 0);
//			LED_RED_ON();
//		}
		else
		{
			LED_RED_OFF();
			motor_data.Angle_out = PID_Plc_Calc(&motor_data.hpid_angle, motor_data.info->angle, angle_set);
			angle_out = motor_data.Angle_out;
			motor_data.Speed_out = PID_Inc_Calc(&motor_data.hpid_speed, motor_data.info->speed_rpm, motor_data.Angle_out);
			speed_out = motor_data.Speed_out;
			motor_data.driver->can_tx_cmd(&hcan1, 0x200, (int16_t)motor_data.Speed_out, 0, 0, 0);
			
//			motor_6020.driver->can_tx_cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), speed, 0, 0, 0);
		}
//		trans(tx_info);
		
		osDelay(1);
	}
	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	switch(rx_header.StdId)
	{
		case RM3508_CAN_ID_201:
		{
			motor_data.Update(&motor_data, rx_data);
	    motor_data.Check(&motor_data);
		  break;
		}
		case GM6020_CAN_ID_205:
		{
			motor_6020.Update(&motor_6020, rx_data);
	    motor_6020.Check(&motor_6020);
		  break;
		}
		case 0x123://任务七部分
		{
			reserve(rx_data);
			break;
		}
		default :
		{
			
		}
	}
	
}

void CAN_Tx_cmd(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data_1, 
								int16_t data_2, int16_t data_3, int16_t data_4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];
	uint32_t tx_mail_box;
	
	tx_header.StdId = identifier;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	
	tx_data[0] = (data_1 >> 8);
	tx_data[1] = data_1;
	tx_data[2] = (data_2 >> 8);
	tx_data[3] = data_2;
	tx_data[4] = (data_3 >> 8);
	tx_data[5] = data_3;
	tx_data[6] = (data_4 >> 8);
	tx_data[7] = data_4;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mail_box);
}

void CAN_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	can_filter_st.FilterActivation = ENABLE;//使能滤波器
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//设置滤波器模式
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//设置比特数
  can_filter_st.FilterIdHigh = 0x0000;//高位ID
  can_filter_st.FilterIdLow = 0x0000;//低位ID
  can_filter_st.FilterMaskIdHigh = 0x0000;//高位掩码
  can_filter_st.FilterMaskIdLow = 0x0000;//低位掩码 注：此配置下没有过滤功能
	
  can_filter_st.FilterBank = 0;//CAN1 过滤器组设置
	can_filter_st.SlaveStartFilterBank = 14;//CAN2 起始过滤器组设置，CAN2是CAN1的Slaver
  HAL_CAN_ConfigFilter(&hcan1,&can_filter_st);//应用配置到CAN1
	HAL_CAN_Start(&hcan1);//启动CAN1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN1中断
	
  can_filter_st.FilterBank = 14;//CAN2 过滤器组设置
	HAL_CAN_ConfigFilter(&hcan2,&can_filter_st);
  HAL_CAN_Start(&hcan2);//启动CAN2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN2中断
}

