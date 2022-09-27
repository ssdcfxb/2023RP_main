#ifndef __CAN_TASK
#define __CAN_TASK

#include "cmsis_os.h"
#include "main.h"
#include "can.h"
#include "motor.h"
#include "6020_motor.h"


void Start_CAN_task(void const * argument);
void CAN_filter_init(void);
void CAN_Tx_cmd(CAN_HandleTypeDef *hcan, uint32_t identifier, int16_t data_1,
								int16_t data_2, int16_t data_3, int16_t data_4);


#endif

