/**
 * @file        drv_can.c
 * @author      RobotPilots
 * @Version     v1.1.1
 * @brief       CAN Driver Package(Based on HAL).
 * @update
 *              v1.0(20-August-2020)
 *              v1.1(26-October-2021)
 *                  1.修改CANx_rxDataHandler()形参canId->rxId
 *              v1.1.1(8-November-2021)
 *                  1.新增邮箱机制
 *              v1.1.2(13-November-2021)
 *                  1.修复创建tx_port的tx_period一直为默认值的bug
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "cmsis_os.h"
#include "rp_math.h"
#include "drv_haltick.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Private macro -------------------------------------------------------------*/
#define DEFAULT_CAN_TX_PERIOD (2)   // 选择自动发送时建议发送间隔>=2ms

/* Private function prototypes -----------------------------------------------*/
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig);
//static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;

/* Exported variables --------------------------------------------------------*/
CAN_MailboxTypeDef hcan1Mailbox = {
    .tx_port = NULL,
    .tx_port_cnt = 0,
    .wait_tx_port_list = {NULL,NULL,NULL,NULL},
    .wait_tx_port_cnt = 0,
};

CAN_MailboxTypeDef hcan2Mailbox = {
    .tx_port = NULL,
    .tx_port_cnt = 0,
    .wait_tx_port_list = {NULL,NULL,NULL,NULL},
    .wait_tx_port_cnt = 0,
};

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	CAN 标识符过滤器复位成默认配置
 */
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// 不过滤
	sFilterConfig->FilterMaskIdLow = 0;						// 不过滤
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// 过滤器关联到FIFO0
	sFilterConfig->FilterBank = 0;							// 设置过滤器0
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// 标识符屏蔽模式
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32位宽
	sFilterConfig->FilterActivation = ENABLE;				// 激活滤波器
	sFilterConfig->SlaveStartFilterBank = 0;
}

/**
 *	@brief	CAN 接收中断回调函数
 */
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
	}
	else if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
	}
}

/**
 *	@brief	CAN 遍历邮箱的发送端口，若存在则返回端口地址
 */
static CAN_TxPortTypeDef* CAN_GetTxPort(CAN_MailboxTypeDef *mailbox, uint32_t tx_id)
{
    bool isFound = false;
    CAN_TxPortTypeDef *port = mailbox->tx_port;
    
    while(port != NULL) {
        if(port->tx_id == tx_id) {
            isFound = true;
            break;
        }
        port = port->next;
    }
    
    if( !isFound )
        port = NULL;
    
    return port;
}

/**
 *	@brief	CAN 创建发送端口
 */
static CAN_TxPortTypeDef* CAN_CreateTxPort(CAN_MailboxTypeDef *mailbox, drv_can_t *drv)
{
    CAN_TxPortTypeDef *port = mailbox->tx_port;
    
    if((port != NULL) && (drv != NULL)) {
        // find the last port
        while(port->next != NULL)
            port = port->next;
        // create a new port(tx_buff fill zero)
        port->next = (CAN_TxPortTypeDef *)pvPortMalloc(sizeof(CAN_MailboxTypeDef));
        port->next->tx_id = drv->tx_id;
        port->next->tx_period = drv->tx_period;
        port->next->last_tx_time = 0;
        for(uint8_t i = 0; i < 8; i++)
            port->next->tx_buff[i] = 0;
        port->next->next = NULL;
        port->next->mailbox = mailbox;
        // return the last created port
        port = port->next;
    }
    else {
        // create a new port(tx_buff fill zero)
        mailbox->tx_port = (CAN_TxPortTypeDef *)pvPortMalloc(sizeof(CAN_MailboxTypeDef));
        mailbox->tx_port->tx_id = drv->tx_id;
        mailbox->tx_port->tx_period = drv->tx_period;
        mailbox->tx_port->last_tx_time = 0;
        for(uint8_t i = 0; i < 8; i++)
            mailbox->tx_port->tx_buff[i] = 0;
        mailbox->tx_port->next = NULL;
        mailbox->tx_port->mailbox = mailbox;
        // return the last created port
        port = mailbox->tx_port;
    }
    
    mailbox->tx_port_cnt += 1;
    
    return port;
}

/**
 *	@brief	CAN 添加消息到发送端口
 */
static void CAN_AddMsgToTxPort(CAN_TxPortTypeDef *port, uint8_t *data, uint8_t data_cnt, uint8_t start_idx)
{
    if(port == NULL) {
        // @TODO: record error msg
        return;
    }
    
    for(uint8_t i = 0; i < data_cnt; i++) {
        if((start_idx+i) >= 8) {
            // @TODO: record error msg
            // error: out of range(must < 8)
            break;
        }
        port->tx_buff[start_idx+i] = data[i];
    }
}

/**
 *	@brief	CAN 更新发送端口的发送间隔(取最短间隔)
 */
static void CAN_UpdatePeriodAtTxPort(CAN_TxPortTypeDef *port, uint16_t tx_period)
{
    if(tx_period < port->tx_period)
        port->tx_period = tx_period;
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	CAN1 初始化
 */
void CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// 配置CAN标识符滤波器
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// 开启CAN1
	HAL_CAN_Start(&hcan1);
}

/**
 *	@brief	CAN2 初始化
 */
void CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// 配置CAN标识符滤波器
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// 开启CAN2
	HAL_CAN_Start(&hcan2);
}

void CAN_AddMsgByDriver(struct drv_can *drv, uint8_t *data, uint8_t data_cnt)
{
    CAN_TxPortTypeDef *port = NULL;
    CAN_MailboxTypeDef *mailbox = NULL;    
     
    if(drv->id == DRV_CAN1) {
        mailbox = &hcan1Mailbox;
    }
    else if(drv->id == DRV_CAN2) {
        mailbox = &hcan2Mailbox;
    }
    else {
        // unknown can_id
        return;
    }
    
    /* send data by mail */
    port = CAN_GetTxPort(mailbox, drv->tx_id);
    /* port not found */
    if(port == NULL) {
        port = CAN_CreateTxPort(mailbox, drv);
    }
    CAN_AddMsgToTxPort(port, data, data_cnt, drv->data_idx);
    CAN_UpdatePeriodAtTxPort(port, drv->tx_period);
}

HAL_StatusTypeDef CAN_StartTxByTxPort(CAN_TxPortTypeDef *txPort)
{
    uint32_t txMailBox;
	CAN_TxHeaderTypeDef txFrameHeader;
    HAL_StatusTypeDef ret = HAL_ERROR; 
    
    if(txPort != NULL) {
        portENTER_CRITICAL();
        
        txFrameHeader.StdId = txPort->tx_id;
        txFrameHeader.IDE = CAN_ID_STD;
        txFrameHeader.RTR = CAN_RTR_DATA;
        txFrameHeader.DLC = 8;
        if((CAN_MailboxTypeDef*)txPort->mailbox == &hcan1Mailbox)
            ret = HAL_CAN_AddTxMessage(&hcan1, &txFrameHeader, &txPort->tx_buff[0], &txMailBox);
        else if((CAN_MailboxTypeDef*)txPort->mailbox == &hcan2Mailbox)
            ret = HAL_CAN_AddTxMessage(&hcan2, &txFrameHeader, &txPort->tx_buff[0], &txMailBox);
        
        txPort->last_tx_time = micros();
        
        portEXIT_CRITICAL();
    }
    
    return ret;
}

HAL_StatusTypeDef CAN_StartTxByDriver(struct drv_can *drv, uint8_t *data)
{
    uint32_t txMailBox;
	CAN_TxHeaderTypeDef txFrameHeader;
    HAL_StatusTypeDef ret = HAL_ERROR;
    
	txFrameHeader.StdId = drv->tx_id;
	txFrameHeader.IDE = CAN_ID_STD;
	txFrameHeader.RTR = CAN_RTR_DATA;
	txFrameHeader.DLC = 8;
    if(drv->id == DRV_CAN1)
        ret = HAL_CAN_AddTxMessage(&hcan1, &txFrameHeader, data, &txMailBox);
    else if(drv->id == DRV_CAN2)
        ret = HAL_CAN_AddTxMessage(&hcan2, &txFrameHeader, data, &txMailBox);
    
    return ret;
}

CAN_TxPortTypeDef* CAN_PopWaitTxPort(CAN_MailboxTypeDef *mailbox)
{
    CAN_TxPortTypeDef* tx_port = NULL;
    
    if((mailbox != NULL) && (mailbox->wait_tx_port_cnt > 0)) {
        tx_port = mailbox->wait_tx_port_list[0];
        for(uint8_t i = 0; i < MAX_WAIT_TX_PORT_CNT-1; i++) {
            mailbox->wait_tx_port_list[i] = mailbox->wait_tx_port_list[i+1];
        }
        mailbox->wait_tx_port_list[MAX_WAIT_TX_PORT_CNT-1] = NULL;
        mailbox->wait_tx_port_cnt -= 1;
    }
    
    return tx_port;
}

HAL_StatusTypeDef CAN_PushWaitTxPort(CAN_MailboxTypeDef *mailbox, CAN_TxPortTypeDef *tx_port)
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    
    if((mailbox != NULL) && (tx_port != NULL)) {
        if(mailbox->wait_tx_port_cnt < MAX_WAIT_TX_PORT_CNT) {
            mailbox->wait_tx_port_list[mailbox->wait_tx_port_cnt] = tx_port;
            mailbox->wait_tx_port_cnt++;
            ret = HAL_OK;
        }
    }
    
    return ret;
}

bool CAN_MailboxReadyForTx(CAN_MailboxTypeDef *mailbox)
{
    bool ret = false;
    CAN_TxPortTypeDef *tx_port = NULL;

    if((mailbox != NULL) && (mailbox->tx_port_cnt > 0)) {
        tx_port = mailbox->tx_port;
        while(tx_port != NULL) {
            uint32_t cur_time = micros();
            if((cur_time - tx_port->last_tx_time) >= (tx_port->tx_period*1000)) {
                CAN_PushWaitTxPort(mailbox, tx_port);
                ret = true;
                //tx_port->last_tx_time = tx_time;
            }
            tx_port = tx_port->next;
        }
    }
    
    return ret;
}

//临时定义变量，后续需要删除
uint32_t send_control_time;

// 以下三个变量临时调试用
//extern uint32_t send_control_time; //后续需要去掉注释
uint32_t tx_time;
int32_t delta_time;
uint32_t dlt_t;
uint32_t last_t;
void CAN_AutoTx(CAN_MailboxTypeDef *mailbox)
{
    CAN_TxPortTypeDef *tx_port = NULL;
    
    if(mailbox != NULL) {
        while(mailbox->wait_tx_port_cnt) {
            tx_port = CAN_PopWaitTxPort(mailbox);
            tx_time = micros();
            delta_time = tx_time - send_control_time;
            dlt_t = micros() - last_t;
            last_t = micros();
            CAN_StartTxByTxPort(tx_port);
            // 可尝试在这里增加小的延时，避免同一个CAN口连续发送
        }
    }
}

/* Callback functions --------------------------------------------------------*/
/**
 *	@brief	重写 CAN RxFifo 中断接收函数
 *	@note	在stm32f4xx_hal_can.c中弱定义
 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	/* CAN1 接收中断 */
//	if(hcan->Instance == CAN1)
//	{
//		CAN_Rx_Callback(hcan);
//		// HAL_CAN_DeactivateNotification
//		// __HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	// 暂停开启FIF00消息挂号中断，在消息处理任务中处理完成后再使能
//	}else
//	if(hcan->Instance == CAN2)
//	{
//		CAN_Rx_Callback(hcan);
//	}
//}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Protocol Layer中实现具体的 CAN1 处理协议
 */
__WEAK void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}

/**
 *	@brief	[__WEAK] 需要在Protocol Layer中实现具体的 CAN2 处理协议
 */
__WEAK void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
}
