#include "CAN_task.h"

int16_t speed = 500;
float current_out = 0.0f;
float speed_out = 0.0f;

void Start_CAN_task(void const * argument)
{
	CAN_filter_init();
	motor_data.Init(&motor_data);
	for(;;)
	{
		if ((motor_data.errno != NONE_ERR) || (motor_data.work_state == DEV_OFFLINE))
		{
			motor_data.driver->can_tx_cmd(&hcan1, 0x200, 0, 0, 0, 0);
			LED_RED_ON();
		}
		else
		{
			LED_RED_OFF();
			motor_data.Angle_out = PID_Plc_Calc(&motor_data.hpid_angle, motor_data.info->speed_rpm, 500.0f);
			speed_out = motor_data.Angle_out;
			motor_data.Speed_out = PID_Plc_Calc(&motor_data.hpid_speed, motor_data.info->given_current, motor_data.Angle_out);
			current_out = motor_data.Speed_out;
			motor_data.driver->can_tx_cmd(&hcan1, 0x200, speed, 0, 0, 0);
		}
		
		osDelay(1);
	}
	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	motor_data.Update(&motor_data, rx_data);
	motor_data.Check(&motor_data);
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
