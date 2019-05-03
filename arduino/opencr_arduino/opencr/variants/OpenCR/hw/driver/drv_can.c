/*
 * drv_can.c
 *
 *  Created on: 2017. 11. 7.
 *      Author: opus
 */

#include <string.h>
#include "drv_can.h"
#include "hw.h"
#include "ring.h"


/* a popular industrial application has optional settings of 125 kbps, 250 kbps, or 500 kbps  */
#define _DRV_CAN_58MHZ_1000KBPS_PRE  3
#define _DRV_CAN_58MHZ_1000KBPS_TS1  CAN_BS1_13TQ
#define _DRV_CAN_58MHZ_1000KBPS_TS2  CAN_BS2_4TQ

#define _DRV_CAN_58MHZ_500KBPS_PRE   6
#define _DRV_CAN_58MHZ_500KBPS_TS1   CAN_BS1_13TQ
#define _DRV_CAN_58MHZ_500KBPS_TS2   CAN_BS2_4TQ

#define _DRV_CAN_58MHZ_250KBPS_PRE   12
#define _DRV_CAN_58MHZ_250KBPS_TS1   CAN_BS1_13TQ
#define _DRV_CAN_58MHZ_250KBPS_TS2   CAN_BS2_4TQ

#define _DRV_CAN_58MHZ_125KBPS_PRE   27
#define _DRV_CAN_58MHZ_125KBPS_TS1   CAN_BS1_11TQ
#define _DRV_CAN_58MHZ_125KBPS_TS2   CAN_BS2_4TQ

#define CAN2_FILTER_BANK_START_NUM   0



typedef struct 
{
  CAN_HandleTypeDef *p_hCANx;
  void (*handler)(void *arg);
  uint8_t rx_fifo;
} drv_can_t;

static CAN_HandleTypeDef   hCAN2;

static ring_node_t ring_msg[DRV_CAN_MAX_CH];
static ring_node_t ring_data[DRV_CAN_MAX_CH];

static drv_can_msg_t can_msg[DRV_CAN_MAX_CH][DRV_CAN_MSG_RX_BUF_MAX];
static uint8_t can_data[DRV_CAN_MAX_CH][DRV_CAN_DATA_RX_BUF_MAX];

static drv_can_t drv_can_tbl[DRV_CAN_MAX_CH] =
{
  {&hCAN2, NULL, CAN_RX_FIFO0}
};


void drvCanInit(void)
{
  uint8_t i;

  for(i = 0; i < DRV_CAN_MAX_CH; i++)
  {
    ringCreate(&ring_msg[i], DRV_CAN_MSG_RX_BUF_MAX);
    ringCreate(&ring_data[i], DRV_CAN_DATA_RX_BUF_MAX);
  }
}

bool drvCanOpen(uint8_t channel, uint32_t baudrate, uint8_t format)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return false;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;
  uint32_t prescale, bs1, bs2;

  switch(baudrate)
  {
    case _DEF_CAN_BAUD_125K :
      prescale = _DRV_CAN_58MHZ_125KBPS_PRE;
      bs1      = _DRV_CAN_58MHZ_125KBPS_TS1;
      bs2      = _DRV_CAN_58MHZ_125KBPS_TS2;
      break;

    case _DEF_CAN_BAUD_250K :
      prescale = _DRV_CAN_58MHZ_250KBPS_PRE;
      bs1      = _DRV_CAN_58MHZ_250KBPS_TS1;
      bs2      = _DRV_CAN_58MHZ_250KBPS_TS2;
      break;

    case _DEF_CAN_BAUD_500K :
      prescale = _DRV_CAN_58MHZ_500KBPS_PRE;
      bs1      = _DRV_CAN_58MHZ_500KBPS_TS1;
      bs2      = _DRV_CAN_58MHZ_500KBPS_TS2;
      break;

    case _DEF_CAN_BAUD_1000K :
      prescale = _DRV_CAN_58MHZ_1000KBPS_PRE;
      bs1      = _DRV_CAN_58MHZ_1000KBPS_TS1;
      bs2      = _DRV_CAN_58MHZ_1000KBPS_TS2;
      break;

    default :
      prescale = _DRV_CAN_58MHZ_125KBPS_PRE;
      bs1      = _DRV_CAN_58MHZ_125KBPS_TS1;
      bs2      = _DRV_CAN_58MHZ_125KBPS_TS2;
      break;
  }

  switch(channel)
  {
    case _DEF_CAN1 :
    case _DEF_CAN2 :
    default :
      p_hCANx->Instance  = CAN2;

      p_hCANx->Init.Mode = CAN_MODE_NORMAL;
      p_hCANx->Init.SyncJumpWidth  = CAN_SJW_1TQ;
      p_hCANx->Init.Prescaler = prescale;
      p_hCANx->Init.TimeSeg1  = bs1;
      p_hCANx->Init.TimeSeg2  = bs2;
      p_hCANx->Init.AutoBusOff = ENABLE;
      p_hCANx->Init.AutoWakeUp = DISABLE;
      p_hCANx->Init.AutoRetransmission = ENABLE;
      p_hCANx->Init.ReceiveFifoLocked = DISABLE;
      p_hCANx->Init.TransmitFifoPriority = DISABLE;
      break;
  }

  if (HAL_CAN_Init(p_hCANx) != HAL_OK)
  {
    return false;
  }

  /* Default Setup Filter */
  drvCanConfigFilter(0, 0x0, 0x0, format);

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(p_hCANx) != HAL_OK)
  {
    return false;
  }

  HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_ERROR);
  HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_LAST_ERROR_CODE);
  HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_BUSOFF);
  HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_ERROR_PASSIVE);
  HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_ERROR_WARNING);

  drvCanAttachRxInterrupt(channel, NULL);

  return true;
}

void drvCanClose(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  drvCanDetachRxInterrupt(channel);
  HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_LAST_ERROR_CODE);
  HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_BUSOFF);
  HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_ERROR_PASSIVE);
  HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_ERROR_WARNING);
  HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_ERROR);

  HAL_CAN_DeInit(p_hCANx);
  HAL_CAN_MspDeInit(p_hCANx);
}

bool drvCanConfigFilter(uint8_t filter_num, uint32_t id, uint32_t mask, uint8_t format)
{
  CAN_FilterTypeDef  sFilterConfig;

  uint32_t reserved;
  uint32_t reg_id;
  uint32_t reg_mask;

  switch(format)
  {
    case _DEF_CAN_STD :
      reserved = _DEF_CAN_STD | CAN_RTR_DATA;
      reg_id   = (id << 21) | reserved;
      reg_mask = (mask << 21) | reserved;
      break;

    case _DEF_CAN_EXT :
    default :
      reserved = _DEF_CAN_EXT | CAN_RTR_DATA;
      reg_id   = ((id << 3) | reserved);// & 0x1FFFFF;
      reg_mask = ((mask << 3) | reserved);// & 0x1FFFFF;
      break;
  }

  sFilterConfig.FilterBank = filter_num;
  sFilterConfig.FilterMode   = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale  = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = reg_id >> 16;
  sFilterConfig.FilterIdLow  = reg_id;
  sFilterConfig.FilterMaskIdHigh = reg_mask >> 16;
  sFilterConfig.FilterMaskIdLow  = reg_mask;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.SlaveStartFilterBank   = CAN2_FILTER_BANK_START_NUM;

  sFilterConfig.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(&hCAN2, &sFilterConfig) != HAL_OK)
  {
    return false;
  }   

  return true;
}

uint32_t drvCanWrite(uint8_t channel, uint32_t id, uint8_t *p_data, uint32_t length, uint8_t format)
{
  if((channel > DRV_CAN_MAX_CH)||(id > 0x1FFFFFFF))
    return 0;

  if(p_data == NULL && length > 0)
    return 0;   

  uint32_t tx_len, sent_len, i;
  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[DRV_CAN_MAX_BYTE_IN_MSG];
  uint32_t tx_mailbox;
    
  switch(format)
  {
    case _DEF_CAN_STD :
      tx_header.IDE   = CAN_ID_STD;
      tx_header.StdId = id;
      break;

    case _DEF_CAN_EXT :
    default :
      tx_header.IDE   = CAN_ID_EXT;
      tx_header.ExtId = id;
      break;
  }

  tx_header.RTR   = CAN_RTR_DATA;

  sent_len = 0;

  while(sent_len < length)
  {
    tx_len = length - sent_len;
    if (tx_len > DRV_CAN_MAX_BYTE_IN_MSG)
    {
      tx_len = DRV_CAN_MAX_BYTE_IN_MSG;
    }

    for(i = 0; i < tx_len; i++)
    {
      tx_data[i] = p_data[sent_len + i];
    }

    tx_header.DLC = tx_len;

    if(HAL_CAN_AddTxMessage(p_hCANx, &tx_header, tx_data, &tx_mailbox) == HAL_OK)
    {
      /* Wait transmission complete */
      while(HAL_CAN_GetTxMailboxesFreeLevel(p_hCANx) != 3);

      sent_len += tx_len;
    }
  }

  if(length == 0)
  {
    tx_header.DLC = 0;

    if(HAL_CAN_AddTxMessage(p_hCANx, &tx_header, tx_data, &tx_mailbox) == HAL_OK)
    {
      /* Wait transmission complete */
      while(HAL_CAN_GetTxMailboxesFreeLevel(p_hCANx) != 3);
    }
  }

  return sent_len;
}


uint8_t drvCanRead(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  uint8_t ret = 0;

  ret = can_data[channel][ringGetReadIndex(&ring_data[channel])];

  ringReadUpdate(&ring_data[channel]);

  return ret;
}

uint32_t drvCanAvailable(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  return ringReadAvailable(&ring_data[channel]);
}

uint32_t drvCanWriteMsg(uint8_t channel, drv_can_msg_t *p_msg)
{
  return drvCanWrite(channel, p_msg->id, p_msg->data, p_msg->length, p_msg->format);
}

drv_can_msg_t* drvCanReadMsg(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return NULL;
  }

  drv_can_msg_t* p_ret = &can_msg[channel][ringGetReadIndex(&ring_msg[channel])];

  ringReadUpdate(&ring_msg[channel]);

  return p_ret;
}

uint32_t drvCanAvailableMsg(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  return ringReadAvailable(&ring_msg[channel]);
}


uint8_t drvCanGetErrCount(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  return (drv_can_tbl[channel].p_hCANx->Instance->ESR) >> 24;
}

uint32_t drvCanGetError(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  return HAL_CAN_GetError(p_hCANx);
}

uint32_t drvCanGetState(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  return HAL_CAN_GetState(p_hCANx);
}

void drvCanAttachRxInterrupt(uint8_t channel, void (*handler)(void *arg))
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  drv_can_tbl[channel].handler = handler;

  switch(drv_can_tbl[channel].rx_fifo)
  {
    case CAN_RX_FIFO0:
      HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_RX_FIFO0_MSG_PENDING);
      break;
      
    case CAN_RX_FIFO1:
      HAL_CAN_ActivateNotification(p_hCANx, CAN_IT_RX_FIFO1_MSG_PENDING);
      break;
      
    default:
      break;
  }
}

void drvCanDetachRxInterrupt(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  drv_can_tbl[channel].handler = NULL;

  switch(drv_can_tbl[channel].rx_fifo)
  {
    case CAN_RX_FIFO0:
      HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_RX_FIFO0_MSG_PENDING);
      break;
      
    case CAN_RX_FIFO1:
      HAL_CAN_DeactivateNotification(p_hCANx, CAN_IT_RX_FIFO1_MSG_PENDING);
      break;
      
    default:
      break;
  }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  uint8_t channel, msg_idx, i;
  drv_can_msg_t *rx_buf;
  uint8_t rx_data[DRV_CAN_MAX_BYTE_IN_MSG];
  CAN_RxHeaderTypeDef rx_header;
  
  for( channel = 0; channel<DRV_CAN_MAX_CH; channel++ )
  {
    if( hcan->Instance == drv_can_tbl[channel].p_hCANx->Instance )
    {
      if (HAL_CAN_GetRxMessage(hcan, drv_can_tbl[channel].rx_fifo, &rx_header, rx_data) == HAL_OK)
      {
        msg_idx = ringGetWriteIndex(&ring_msg[channel]);  
        rx_buf  = &can_msg[channel][msg_idx];

        if(rx_header.IDE == CAN_ID_STD)
        {
          rx_buf->id = rx_header.StdId;  
          rx_buf->format = _DEF_CAN_STD;
        }
        else
        {
          rx_buf->id = rx_header.ExtId;
          rx_buf->format = _DEF_CAN_EXT;
        }
        rx_buf->length = rx_header.DLC;
        memcpy(rx_buf->data, rx_data, rx_buf->length);
        ringWriteUpdate(&ring_msg[channel]);

        if( drv_can_tbl[channel].handler != NULL )
        {
          (*drv_can_tbl[channel].handler)((void *)rx_buf);
          ringReadUpdate(&ring_msg[channel]);
        }
        else  //store byte data
        {
          for(i = 0; i < rx_buf->length; i++)
          {
            can_data[channel][ringGetWriteIndex(&ring_data[channel])] = rx_buf->data[i];
            ringWriteUpdate(&ring_data[channel]);
          }
        }
      }      
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  UNUSED(hcan);
}

void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hCAN2);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(hcan->Instance==CAN2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 4, 1);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN2)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    __HAL_RCC_CAN2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  }
}
