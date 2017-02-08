/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_BUF_SIZE   (1024*16)
#define APP_RX_DATA_SIZE  (1024)
#define APP_TX_DATA_SIZE  (1024)



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;
volatile uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
volatile uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */

static BOOL is_opened = FALSE;
static BOOL is_reopen = FALSE;

volatile uint8_t  rxd_buffer[APP_RX_BUF_SIZE];
volatile uint32_t rxd_length    = 0;
volatile uint32_t rxd_BufPtrIn  = 0;
volatile uint32_t rxd_BufPtrOut = 0;


/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);




USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  USBD_SetupReqTypedef *req = (USBD_SetupReqTypedef *)pbuf;


  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];

    is_opened = TRUE;
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    is_opened = req->wValue&0x01;
    is_reopen = TRUE;
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
}


/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
  uint32_t i;


  for( i=0; i<*Len; i++ )
  {
    rxd_buffer[rxd_BufPtrIn] = Buf[i];

    rxd_BufPtrIn++;

    /* To avoid buffer overflow */
    if(rxd_BufPtrIn == APP_RX_BUF_SIZE)
    {
      rxd_BufPtrIn = 0;
    }
  }


  USBD_CDC_ReceivePacket(&USBD_Device);
  return (USBD_OK);
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_Write
     WORK    :
---------------------------------------------------------------------------*/
void CDC_Itf_Write( uint8_t *p_buf, uint32_t length )
{
  uint32_t i;
  uint32_t remain_length = 0;
  uint32_t tx_length = 0;
  uint32_t write_length;
  uint32_t written_length;
  uint32_t tTime;
  uint32_t time_out = 1000;
  uint8_t  ret;
  BOOL timeout_expired = FALSE;


  if( USBD_Device.pClassData == NULL ) return;
  if( is_opened == FALSE && is_reopen == FALSE ) return;
  is_reopen = FALSE;


  written_length = 0;
  while(1)
  {
    write_length = length - written_length;

    if( write_length > APP_TX_DATA_SIZE )  write_length = APP_TX_DATA_SIZE;
    memcpy( UserTxBuffer, &p_buf[written_length], write_length );

    USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, write_length);

    tTime = millis();
    while(1)
    {
      ret = USBD_CDC_TransmitPacket(&USBD_Device);
      if(ret == USBD_OK)
      {
        written_length += write_length;
        is_opened = TRUE;
        break;
      }
      else if(ret == USBD_BUSY)
      {
        if( (millis()-tTime) > time_out )
        {
          if( is_reopen == FALSE )
          {
            is_opened = FALSE;
          }
          timeout_expired = TRUE;
          break;
        }

        if(USBD_Device.dev_state != USBD_STATE_CONFIGURED)
        {
          is_opened = FALSE;
          break;
        }
      }
      else
      {
        is_opened = FALSE;
        break;
      }
    }

    if( is_opened       == FALSE ) break;
    if( timeout_expired == TRUE )  break;
    if( written_length >= length ) break;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_IsAvailable
     WORK    :
---------------------------------------------------------------------------*/
BOOL CDC_Itf_IsAvailable( void )
{
  if( rxd_BufPtrIn != rxd_BufPtrOut ) return TRUE;

  return FALSE;
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_Getch
     WORK    :
---------------------------------------------------------------------------*/
uint8_t CDC_Itf_Getch( void )
{
  uint8_t ch = 0;
  uint32_t buffptr;


  while(1)
  {
    if( CDC_Itf_IsAvailable() ) break;
  }



  buffptr = rxd_BufPtrOut;

  ch = rxd_buffer[buffptr];

  __disable_irq();
  rxd_BufPtrOut += 1;
  if (rxd_BufPtrOut == APP_RX_BUF_SIZE)
  {
    rxd_BufPtrOut = 0;
  }
  __enable_irq();

  return ch;
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
