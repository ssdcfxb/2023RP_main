#include "vision_protocol.h"

extern UART_HandleTypeDef huart1;

vision_tx_info_t vision_tx_info = {
	.SOF = 0xA5,
};
vision_rx_info_t vision_rx_info;
uint8_t vision_txBuf[30];

bool vision_send_data(void)
{
	memcpy(vision_txBuf, &vision_tx_info, 3);
	Append_CRC8_Check_Sum(vision_txBuf, 3);
	Append_CRC16_Check_Sum(vision_txBuf, sizeof(vision_tx_info_t));
	
	if(HAL_UART_Transmit_DMA(&huart1,vision_txBuf,sizeof(vision_tx_info_t)) == HAL_OK)
	{
			return true;
	}
	return false;
}

bool vision_recieve_data(uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(vision_rx_info_t)) == true)
			{
				memcpy(&vision_rx_info, rxBuf, sizeof(vision_rx_info_t));
				return true;
			}
		}
	}
	return false;
}

void USART1_rxDataHandler(uint8_t *rxBuf)
{
	
}
