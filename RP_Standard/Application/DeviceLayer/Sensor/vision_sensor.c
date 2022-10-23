#include "vision_sensor.h"

void vision_init(vision_sensor_t *vis_sen);
void vision_update(vision_sensor_t *vis_sen, uint8_t *rxBuf);
void vision_check(vision_sensor_t *vis_sen);	
void vision_heart_beat(vision_sensor_t *vis_sen);

drv_uart_t	vision_sensor_driver = {
	.id = DRV_UART1,
};

vision_info_t 	vision_sensor_info = {
	.tx_info = &vision_tx_info,
	.rx_info = &vision_rx_info,
	.offline_max_cnt = 200,
};

vision_sensor_t vision_sensor = {
	.info = &vision_sensor_info,
	.driver = &vision_sensor_driver,
	.init = vision_init,
	.update = vision_update,
	.check = vision_check,
	.heart_beat = vision_heart_beat,
	.work_state = DEV_OFFLINE,
	.id = DEV_ID_VISION,
};

void vision_init(vision_sensor_t *vis_sen)
{
	vision_tx_info_t *tx_info = vis_sen->info->tx_info;
	vision_rx_info_t *rx_info = vis_sen->info->rx_info;
	
	vis_sen->info->rx_flag = 0;
	
	tx_info->dataf_1 = 0.0f;
	tx_info->dataf_2 = 0.0f;
	tx_info->datau8_1 = 0; // mode
	tx_info->datau8_2 = 0;
	tx_info->datau8_3 = 0;
	
	rx_info->dataf_1 = 0.0f;
	rx_info->dataf_2 = 0.0f;
	rx_info->datau8_1 = 0; // mode
	rx_info->datau8_2 = 0;
	rx_info->datau8_3 = 0;
	rx_info->datau8_4 = 0;
}

void vision_update(vision_sensor_t *vis_sen, uint8_t *rxBuf)
{
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, 3) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf, sizeof(vision_rx_info_t)) == true)
			{
				memcpy(&vision_rx_info, rxBuf, sizeof(vision_rx_info_t));
				vis_sen->info->rx_flag = 1;
				return;
			}
		}
	}
	vis_sen->info->rx_flag = 0;
}
void vision_check(vision_sensor_t *vis_sen)
{
	vision_rx_info_t *rx_info = vis_sen->info->rx_info;
	vision_info_t *info = vis_sen->info;
	
	info->offline_cnt = 0;
	
	if (info->rx_flag == 1)
	{
		memcpy(&info->cmd_mode, &rx_info->datau8_1, 1);
		memcpy(&info->target_pitch_angle, (void*)&rx_info->dataf_1, 4);
		memcpy(&info->target_yaw_angle, (void*)&rx_info->dataf_2, 4);
		memcpy(&info->is_find_target, &rx_info->datau8_2, 1);
		memcpy(&info->is_find_defund, &rx_info->datau8_3, 1);
		memcpy(&info->is_hit_enable, &rx_info->datau8_4, 1);
	}
}
void vision_heart_beat(vision_sensor_t *vis_sen)
{
	vision_info_t *vis_info = vis_sen->info;

	vis_info->offline_cnt++;
	if(vis_info->offline_cnt > vis_info->offline_max_cnt) {
		vis_info->offline_cnt = vis_info->offline_max_cnt;
		vis_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* ÀëÏß->ÔÚÏß */
		if(vis_sen->work_state == DEV_OFFLINE)
		{
			vis_sen->work_state = DEV_ONLINE;
		}
	}
}



