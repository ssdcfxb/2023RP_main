#include "drv_can.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t can1_tx_buf[16];
uint8_t can2_tx_buf[16];
can_rx_info_t CAN_RxInfo;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInfo.Header, CAN_RxInfo.Data);
	if(hcan == &hcan1)
  {
    CAN1_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else if(hcan == &hcan2)
  {
    CAN2_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else 
  {
    return;
  }
}

HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
	CAN_TxHeaderTypeDef pHeader;
	uint32_t txMailBox;
	
	if((hcan->Instance != CAN1)&&(hcan->Instance != CAN2))
	{
		return HAL_ERROR;
	}
	
	pHeader.StdId = stdId;
	pHeader.IDE = CAN_ID_STD;
	pHeader.RTR = CAN_RTR_DATA;
	pHeader.DLC = 8;
	
	if(HAL_CAN_AddTxMessage(hcan, &pHeader, dat, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	
	return HAL_OK;
}

void CAN_Tx_Cmd(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data_1, 
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

void CAN_Filter_Init(void)
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

