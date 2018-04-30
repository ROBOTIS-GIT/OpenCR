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
#define _DRV_CAN_58MHZ_1000KBPS_PRE  18
#define _DRV_CAN_58MHZ_1000KBPS_TS1  CAN_BS1_1TQ
#define _DRV_CAN_58MHZ_1000KBPS_TS2  CAN_BS2_1TQ

#define _DRV_CAN_58MHZ_500KBPS_PRE   18
#define _DRV_CAN_58MHZ_500KBPS_TS1   CAN_BS1_3TQ
#define _DRV_CAN_58MHZ_500KBPS_TS2   CAN_BS2_2TQ

#define _DRV_CAN_58MHZ_250KBPS_PRE   18
#define _DRV_CAN_58MHZ_250KBPS_TS1   CAN_BS1_8TQ
#define _DRV_CAN_58MHZ_250KBPS_TS2   CAN_BS2_3TQ

#define _DRV_CAN_58MHZ_125KBPS_PRE   18
#define _DRV_CAN_58MHZ_125KBPS_TS1   CAN_BS1_16TQ
#define _DRV_CAN_58MHZ_125KBPS_TS2   CAN_BS2_7TQ

#define CAN2_FILTER_BANK_START_NUM   14



typedef struct 
{
  CAN_HandleTypeDef *p_hCANx;
  void (*handler)(void *arg);
  uint8_t rx_fifo;
} drv_can_t;

static CAN_HandleTypeDef  hCAN1, hCAN2;
static CanTxMsgTypeDef    TxMessage[DRV_CAN_MAX_CH];
static CanRxMsgTypeDef    RxMessage[DRV_CAN_MAX_CH];

static ring_node_t ring_msg[DRV_CAN_MAX_CH];
static ring_node_t ring_data[DRV_CAN_MAX_CH];

static drv_can_msg_t can_msg[DRV_CAN_MAX_CH][DRV_CAN_MSG_RX_BUF_MAX];
static uint8_t can_data[DRV_CAN_MAX_CH][DRV_CAN_DATA_RX_BUF_MAX];

static drv_can_t drv_can_tbl[DRV_CAN_MAX_CH] =
{
  {&hCAN1, NULL, CAN_FIFO0},
  {&hCAN2, NULL, CAN_FIFO1}
};

static uint8_t msg_format = _DEF_CAN_EXT;


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

  if((format != _DEF_CAN_STD)&&(format != _DEF_CAN_EXT))
  {
    format = _DEF_CAN_EXT;
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

    case _DEF_CAN_BAUD_1M :
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
      p_hCANx->Instance  = CAN1;
      p_hCANx->Init.Prescaler = prescale;
      p_hCANx->Init.Mode = CAN_MODE_NORMAL;
      p_hCANx->Init.SJW  = CAN_SJW_1TQ;
      p_hCANx->Init.BS1  = bs1;
      p_hCANx->Init.BS2  = bs2;
      p_hCANx->Init.TTCM = DISABLE;
      p_hCANx->Init.ABOM = DISABLE;
      p_hCANx->Init.AWUM = DISABLE;
      p_hCANx->Init.NART = ENABLE;
      p_hCANx->Init.RFLM = DISABLE;
      p_hCANx->Init.TXFP = DISABLE;
      break;

    case _DEF_CAN2 :
      p_hCANx->Instance  = CAN2;
      p_hCANx->Init.Prescaler = prescale;
      p_hCANx->Init.Mode = CAN_MODE_NORMAL;
      p_hCANx->Init.SJW  = CAN_SJW_1TQ;
      p_hCANx->Init.BS1  = bs1;
      p_hCANx->Init.BS2  = bs2;
      p_hCANx->Init.TTCM = DISABLE;
      p_hCANx->Init.ABOM = DISABLE;
      p_hCANx->Init.AWUM = DISABLE;
      p_hCANx->Init.NART = ENABLE;
      p_hCANx->Init.RFLM = DISABLE;
      p_hCANx->Init.TXFP = DISABLE;
      break;

    default :
      return false;
  }

  p_hCANx->pTxMsg = &TxMessage[channel];
  p_hCANx->pRxMsg = &RxMessage[channel];

  if (HAL_CAN_Init(p_hCANx) != HAL_OK)
  {
    return false;
  }

  msg_format = format;

  /* Default Setup Filter */
  if(p_hCANx->Instance == CAN1)
  {
    drvCanConfigFilter(0, 0x0, 0x0);
  }
  else
  {
    drvCanConfigFilter(CAN2_FILTER_BANK_START_NUM, 0x0, 0x0);
  }

  HAL_CAN_Receive_IT(p_hCANx, drv_can_tbl[channel].rx_fifo);

  return true;
}

void drvCanClose(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  HAL_CAN_DeInit(p_hCANx);
  HAL_CAN_MspDeInit(p_hCANx);
}

bool drvCanConfigFilter(uint8_t filter_num, uint32_t id, uint32_t mask)
{
  CAN_FilterConfTypeDef  sFilterConfig;

  uint32_t reserved;
  uint32_t reg_id;
  uint32_t reg_mask;

  switch(msg_format)
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

  sFilterConfig.FilterNumber = filter_num;
  sFilterConfig.FilterMode   = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale  = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = reg_id >> 16;
  sFilterConfig.FilterIdLow  = reg_id;
  sFilterConfig.FilterMaskIdHigh = reg_mask >> 16;
  sFilterConfig.FilterMaskIdLow  = reg_mask;
  if(filter_num < CAN2_FILTER_BANK_START_NUM)
  {
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  }
  else
  {
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  }
  sFilterConfig.BankNumber   = CAN2_FILTER_BANK_START_NUM;

  sFilterConfig.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(&hCAN1, &sFilterConfig) != HAL_OK)
    return false;

  return true;
}

uint32_t drvCanWrite(uint8_t channel, uint32_t id, uint8_t *p_data, uint32_t length)
{
  if((channel > DRV_CAN_MAX_CH)||(id > 0x1FFFFFFF))
    return 0;

  uint32_t tx_len, sent_len, i;
  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  switch(msg_format)
  {
    case _DEF_CAN_STD :
      p_hCANx->pTxMsg->IDE   = CAN_ID_STD;
      p_hCANx->pTxMsg->StdId = id;
      break;

    case _DEF_CAN_EXT :
    default :
      p_hCANx->pTxMsg->IDE   = CAN_ID_EXT;
      p_hCANx->pTxMsg->ExtId = id;
      break;
  }

  p_hCANx->pTxMsg->RTR   = CAN_RTR_DATA;

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
      p_hCANx->pTxMsg->Data[i] = p_data[sent_len + i];
    }

    p_hCANx->pTxMsg->DLC = tx_len;

    if(HAL_CAN_Transmit(p_hCANx, 10) != HAL_OK)
    {
      break;
    }

    sent_len += tx_len;
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
  return drvCanWrite(channel, p_msg->id, p_msg->data, p_msg->length);
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

  return p_hCANx->ErrorCode;
}

uint32_t drvCanGetState(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return 0;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  return p_hCANx->State;
}

void drvCanAttachRxInterrupt(uint8_t channel, void (*handler)(void *arg))
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  CAN_HandleTypeDef *p_hCANx = drv_can_tbl[channel].p_hCANx;

  drv_can_tbl[channel].handler = handler;

  HAL_CAN_Receive_IT(p_hCANx, drv_can_tbl[channel].rx_fifo);
}

void drvCanDetachRxInterrupt(uint8_t channel)
{
  if(channel > DRV_CAN_MAX_CH)
  {
    return;
  }

  drv_can_tbl[channel].handler = NULL;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  uint8_t channel, msg_idx, i;
  drv_can_msg_t *rx_buf;
  CanRxMsgTypeDef *p_RxMsg;

  for( channel = 0; channel<DRV_CAN_MAX_CH; channel++ )
  {
    if( hcan->Instance == drv_can_tbl[channel].p_hCANx->Instance )
    {
      p_RxMsg = drv_can_tbl[channel].p_hCANx->pRxMsg;
      msg_idx = ringGetWriteIndex(&ring_msg[channel]);
      rx_buf  = &can_msg[channel][msg_idx];

      rx_buf->id = p_RxMsg->ExtId;
      rx_buf->length = p_RxMsg->DLC;
      memcpy(rx_buf->data, p_RxMsg->Data, rx_buf->length);
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

      HAL_CAN_Receive_IT(hcan, drv_can_tbl[channel].rx_fifo);
    }
  }
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(drv_can_tbl[_DEF_CAN1].p_hCANx);
}

void CAN2_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(drv_can_tbl[_DEF_CAN2].p_hCANx);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(hcan->Instance==CAN1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }

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

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 4, 1);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  }

  if(hcan->Instance==CAN2)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    __HAL_RCC_CAN2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  }
}
