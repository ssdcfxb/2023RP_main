/**
 * @file        can_protocol.c
 * @author      RobotPilots
 * @Version     v1.1.2
 * @brief       CAN Protocol.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.�޸�can_potocol.c/.h->can_protocol.c/.h
 *              v1.1.1(8-November-2021)
 *                  1.����can������Ϣ����Э�� 
 *              v1.1.2(13-November-2021)
 *                  1.ȥ��add_halfword��add_word�����������ǿ��ת��������ԭ��
 *                    add_word�д���ת����������⣬�����޸�
 */
 
/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"

#include "drv_can.h"
//#include "chassis_motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	CAN �����Ϣ
 */
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_AddMsgByDriver(drv, data, data_cnt);
}

/**
 *	@brief	CAN ��ӵ��ֽ�����(Byte)
 */
void CAN_AddByte(drv_can_t *drv, uint8_t byte)
{
    CAN_AddMsgByDriver(drv, &byte, 1);
}

/**
 *	@brief	CAN ��Ӱ�������(HalfWord)
 */
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword)
{
    uint8_t data[2];
    data[0] = (uint8_t)(hword >> 8);
    data[1] = (uint8_t)(hword);
	CAN_AddMsgByDriver(drv, data, 2);
}

/**
 *	@brief	CAN ���������(Word)
 */
void CAN_AddWord(drv_can_t *drv, uint32_t word)
{
    uint8_t data[4];
    data[0] = (uint8_t)(word >> 24);
    data[1] = (uint8_t)(word >> 16);
    data[2] = (uint8_t)(word >> 8);
    data[3] = (uint8_t)(word);
	CAN_AddMsgByDriver(drv, data, 4);
}

/**
*	@brief	CAN �������ͱ���
 */
void CAN_ManualTx(drv_can_t *drv, uint8_t *data)
{
    CAN_StartTxByDriver(drv, data);
}

///**
// *	@brief	CAN1 ��������
// */
//void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
//{
//	/* ��ǰ�� */
//	if(rxId == CHASSIS_CAN_ID_LF)
//	{
//		// ���µ��̵������
//		chassis_motor[CHAS_LF].update(&chassis_motor[CHAS_LF], rxBuf);
//		chassis_motor[CHAS_LF].check(&chassis_motor[CHAS_LF]);
//	}
//	/* ��ǰ�� */
//	else if(rxId == CHASSIS_CAN_ID_RF)
//	{
//		// ���µ��̵������
//		chassis_motor[CHAS_RF].update(&chassis_motor[CHAS_RF], rxBuf);
//		chassis_motor[CHAS_RF].check(&chassis_motor[CHAS_RF]);
//	}
//	/* ����� */
//	else if(rxId == CHASSIS_CAN_ID_LB)
//	{
//		// ���µ��̵������
//		chassis_motor[CHAS_LB].update(&chassis_motor[CHAS_LB], rxBuf);
//		chassis_motor[CHAS_LB].check(&chassis_motor[CHAS_LB]);
//	}
//	/* �Һ��� */
//	else if(rxId == CHASSIS_CAN_ID_RB)
//	{
//		// ���µ��̵������
//		chassis_motor[CHAS_RB].update(&chassis_motor[CHAS_RB], rxBuf);
//		chassis_motor[CHAS_RB].check(&chassis_motor[CHAS_RB]);
//	}
//}

/**
 *	@brief	CAN2 ��������
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
}

