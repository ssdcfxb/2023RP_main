#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "device.h"
#include "driver.h"
#include "rm_protocol.h"

void CAN1_Get_Data(uint32_t identifier, uint8_t *data); //CAN1接收函数
void CAN2_Get_Data(uint32_t identifier, uint8_t *data); //CAN2接收函数
void CAN_Send_All(void);

#endif
