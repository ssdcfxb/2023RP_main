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

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	vision_sensor.update(&vision_sensor, rxBuf);
	vision_sensor.check(&vision_sensor);
}
