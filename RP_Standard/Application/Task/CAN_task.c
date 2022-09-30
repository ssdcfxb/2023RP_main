#include "CAN_task.h"

float angle_set = 0.0f;
float speed_out = 0.0f;
float angle_out = 0.0f;

void Start_CAN_task(void const * argument)
{
	CAN_filter_init();
	chassis_motor[CHAS_LF].init(&chassis_motor[CHAS_LF]);
	motor_6020.init(&motor_6020);
	
	for(;;)
	{
		if ((chassis_motor[CHAS_LF].errno != NONE_ERR) || (chassis_motor[CHAS_LF].work_state == DEV_OFFLINE))
		{
			chassis_motor[CHAS_LF].driver->can_tx_cmd(&hcan1, RM3508_GetTxId(chassis_motor[CHAS_LF].driver), 0, 0, 0, 0);
		}
//		if ((motor_6020.errno != NONE_ERR) || (motor_6020.work_state == DEV_OFFLINE))
//		{
//			motor_data.driver->can_tx_cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), 0, 0, 0, 0);
//		}
		else
		{
			LED_RED_OFF();
			chassis_motor[CHAS_LF].Angle_out = PID_Plc_Calc(&chassis_motor[CHAS_LF].hpid_angle, chassis_motor[CHAS_LF].info->angle, angle_set);
			angle_out = chassis_motor[CHAS_LF].Angle_out;
			chassis_motor[CHAS_LF].Speed_out = PID_Inc_Calc(&chassis_motor[CHAS_LF].hpid_speed, chassis_motor[CHAS_LF].info->speed_rpm, chassis_motor[CHAS_LF].Angle_out);
			speed_out = chassis_motor[CHAS_LF].Speed_out;
			chassis_motor[CHAS_LF].driver->can_tx_cmd(&hcan1, 0x200, (int16_t)chassis_motor[CHAS_LF].Speed_out, 0, 0, 0);
			
//			motor_6020.driver->can_tx_cmd(&hcan1, GM6020_GetTxId(motor_6020.driver), speed, 0, 0, 0);
		}
		
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
			chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], rx_data);
	    chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
		  break;
		}
		case GM6020_CAN_ID_205:
		{
			motor_6020.update(&motor_6020, rx_data);
	    motor_6020.check(&motor_6020);
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
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//ʹ��FIFO0
	can_filter_st.FilterActivation = ENABLE;//ʹ���˲���
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//�����˲���ģʽ
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//���ñ�����
  can_filter_st.FilterIdHigh = 0x0000;//��λID
  can_filter_st.FilterIdLow = 0x0000;//��λID
  can_filter_st.FilterMaskIdHigh = 0x0000;//��λ����
  can_filter_st.FilterMaskIdLow = 0x0000;//��λ���� ע����������û�й��˹���
	
  can_filter_st.FilterBank = 0;//CAN1 ������������
	can_filter_st.SlaveStartFilterBank = 14;//CAN2 ��ʼ�����������ã�CAN2��CAN1��Slaver
  HAL_CAN_ConfigFilter(&hcan1,&can_filter_st);//Ӧ�����õ�CAN1
	HAL_CAN_Start(&hcan1);//����CAN1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ��CAN1�ж�
	
  can_filter_st.FilterBank = 14;//CAN2 ������������
	HAL_CAN_ConfigFilter(&hcan2,&can_filter_st);
  HAL_CAN_Start(&hcan2);//����CAN2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ��CAN2�ж�
}

