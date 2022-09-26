/**
 * @file        drv_can.h
 * @author      RobotPilots
 * @Version     v1.1.1
 * @brief       CAN Driver Package(Based on HAL).
 * @update
 *              v1.0(20-August-2020)
 *              v1.1(26-October-2021)
 *                  1.�޸�CANx_rxDataHandler()�β�canId->rxId
 *              v1.1.1(8-November-2021)
 *                  1.�����������
 *              v1.1.2(13-November-2021)
 *                  1.�޸�����tx_port��tx_periodһֱΪĬ��ֵ��bug
 */
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "rp_driver_config.h"
/* Exported macro ------------------------------------------------------------*/
#define MAX_WAIT_TX_PORT_CNT 4
/* Exported types ------------------------------------------------------------*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;

typedef struct CAN_RxPortType {
    uint32_t rx_id;
    uint8_t  rx_buff[8];
    struct CAN_RxPortType *next;
} CAN_RxPortTypeDef;

typedef struct CAN_TxPortType {
    uint32_t tx_id;
    uint8_t  tx_buff[8];
    uint16_t tx_period;
    uint32_t last_tx_time;
    void     *mailbox;
    struct CAN_TxPortType *next;
} CAN_TxPortTypeDef;

typedef struct {
    //CAN_RxPortTypeDef *rx_port;
    CAN_TxPortTypeDef *tx_port;
    uint8_t            tx_port_cnt;
    CAN_TxPortTypeDef *wait_tx_port_list[MAX_WAIT_TX_PORT_CNT];
    uint8_t            wait_tx_port_cnt;
    // MailboxState
} CAN_MailboxTypeDef;

extern CAN_MailboxTypeDef hcan1Mailbox;
extern CAN_MailboxTypeDef hcan2Mailbox;

/* Exported functions --------------------------------------------------------*/
void CAN1_Init(void);
void CAN2_Init(void);
//uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);
//uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat);
//uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat);
HAL_StatusTypeDef CAN_StartTxByTxPort(CAN_TxPortTypeDef *txPort);
HAL_StatusTypeDef CAN_StartTxByDriver(struct drv_can *drv, uint8_t *data);
void CAN_AddMsgByDriver(struct drv_can *drv, uint8_t *data, uint8_t data_cnt);
CAN_TxPortTypeDef* CAN_PopWaitTxPort(CAN_MailboxTypeDef *mailbox);
HAL_StatusTypeDef CAN_PushWaitTxPort(CAN_MailboxTypeDef *mailbox, CAN_TxPortTypeDef *tx_port);
bool CAN_MailboxReadyForTx(CAN_MailboxTypeDef *mailbox);
void CAN_AutoTx(CAN_MailboxTypeDef *mailbox);

#endif
