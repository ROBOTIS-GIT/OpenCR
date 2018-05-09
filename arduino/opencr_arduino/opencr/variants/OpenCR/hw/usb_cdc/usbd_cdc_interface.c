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
#include "wdg.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_BUF_SIZE   (1024*16)
#define APP_RX_DATA_SIZE  (1024*2)
#define APP_TX_DATA_SIZE  (1024*2)


const char *JUMP_BOOT_STR = "OpenCR 5555AAAA";


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};


uint8_t CDC_Reset_Status = 0;
uint8_t CDC_Reset_Status_Baud = 0;

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint8_t UserTxBufferForUSB[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */

uint32_t BuffLength;
static uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
static uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
static uint16_t UserTxBufPtrOutShadow = 0; // shadow of above
static uint8_t  UserTxBufPtrWaitCount = 0; // used to implement a timeout waiting for low-level USB driver
static uint8_t  UserTxNeedEmptyPacket = 0; // used to flush the USB IN endpoint if the last packet was exactly the endpoint packet size

static BOOL is_opened = FALSE;
static BOOL is_reopen = FALSE;
volatile bool usb_rx_full = false;


static uint8_t  rxd_buffer[APP_RX_BUF_SIZE];
static uint32_t rxd_length    = 0;
static uint32_t rxd_BufPtrIn  = 0;
static uint32_t rxd_BufPtrOut = 0;

uint32_t usb_cdc_debug_cnt[16] = {0,};


/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
       void   CDC_Itf_TxISR(void);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);
static uint32_t CDC_Itf_TxAvailable( void );



USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

uint32_t usb_cdc_bitrate = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBufferForUSB, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
  is_opened = FALSE;
  LineCoding.bitrate = 0;
  usb_cdc_bitrate = 0;

  BuffLength            = 0;
  UserTxBufPtrIn        = 0;
  UserTxBufPtrOut       = 0;
  UserTxBufPtrOutShadow = 0;
  UserTxBufPtrWaitCount = 0;
  UserTxNeedEmptyPacket = 0;

  rxd_length            = 0;
  rxd_BufPtrIn          = 0;
  rxd_BufPtrOut         = 0;

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
  is_opened = FALSE;
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
  UNUSED(length);
  
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

    usb_cdc_bitrate = LineCoding.bitrate;

    if( LineCoding.bitrate == 1200 )
    {
        CDC_Reset_Status_Baud = 1;
    }
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
    if( req->wValue & 0x02 )
    {
      CDC_Reset_Status = 1;
    }
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

void CDC_Itf_SofISR(void)
{
  uint32_t rx_buf_length;


  rx_buf_length = APP_RX_DATA_SIZE - CDC_Itf_Available() - 1;

  // 수신버퍼가 USB 전송 패킷 이상 남았을때만 수신하도록 함.
  if (usb_rx_full == true)
  {
    if (rx_buf_length > CDC_DATA_FS_MAX_PACKET_SIZE)
    {
      USBD_CDC_ReceivePacket(&USBD_Device);
      usb_rx_full = false;
    }
  }
  CDC_Itf_TxISR();
}

void CDC_Itf_TxISR(void)
{
  uint32_t buffptr;
  uint32_t buffsize;
  USBD_CDC_HandleTypeDef   *hcdc = USBD_Device.pClassData;

  if(hcdc == NULL)
  {
    return;
  }
  if(hcdc->TxState != 0)
  {
    return;
  }

  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
    if(UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
    {
      buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
    }
    else
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }

    // TODO: 보낼데이터가 64의 배수이면 제로패킷을 보내야 해서, 64의 배수가 되지 않도록 임식 변경
    if (buffsize%CDC_DATA_FS_MAX_PACKET_SIZE == 0 && buffsize > 0)
    {
      buffsize -= 1;
    }

    buffptr = UserTxBufPtrOut;

    memcpy(UserTxBufferForUSB, (uint8_t*)&UserTxBuffer[buffptr], buffsize);
    USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBufferForUSB, buffsize);

    if(USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
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
  uint32_t rx_buf_length;


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

  if( CDC_Reset_Status == 1 )
  {
    CDC_Reset_Status = 0;

    if( *Len >= 15 )
    {
      for( i=0; i<15; i++ )
      {
        if( JUMP_BOOT_STR[i] != Buf[i] ) break;
      }

      if( i == 15 )
      {
        wdg_setup(10);
        wdg_start();
      }
    }
  }

  if( CDC_Reset_Status_Baud )
  {
    wdg_setup(10);
    wdg_start();
  }


  rx_buf_length = APP_RX_DATA_SIZE - CDC_Itf_Available() - 1;

  // 수신버퍼가 USB 전송 패킷 이상 남았을때만 수신하도록 함.
  if (rx_buf_length > CDC_DATA_FS_MAX_PACKET_SIZE)
  {
    USBD_CDC_ReceivePacket(&USBD_Device);
  }
  else
  {
    usb_rx_full = true;
  }
  return (USBD_OK);
}

#if 1
int32_t CDC_Itf_Write( uint8_t *p_buf, uint32_t length )
{
  uint32_t i;
  uint32_t ptr_index;


  if( USBD_Device.pClassData == NULL )
  {
    return -1;
  }
  if( is_opened == FALSE && is_reopen == FALSE )
  {
    return -1;
  }
  if( USBD_Device.dev_state != USBD_STATE_CONFIGURED )
  {
    return -1;
  }
  if (length >= CDC_Itf_TxAvailable())
  {
    return 0;
  }

  __disable_irq();

  ptr_index = UserTxBufPtrIn;


  for (i=0; i<length; i++)
  {
    UserTxBuffer[ptr_index] = p_buf[i];

    ptr_index++;

    /* To avoid buffer overflow */
    if(ptr_index == APP_TX_DATA_SIZE)
    {
      ptr_index = 0;
    }
  }
  UserTxBufPtrIn = ptr_index;
  __enable_irq();

  return length;
}
#else
/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_Write
     WORK    :
---------------------------------------------------------------------------*/
void CDC_Itf_Write( uint8_t *p_buf, uint32_t length )
{
  uint32_t timeout = 500;


  if( is_opened == FALSE ) return;

  is_reopen = FALSE;

  for (uint32_t i = 0; i < length; i++) {
      // Wait until the device is connected and the buffer has space, with a given timeout
      uint32_t start = millis();
      while (is_opened == FALSE || ((UserTxBufPtrIn + 1) & (APP_TX_DATA_SIZE - 1)) == UserTxBufPtrOut) {
          // Wraparound of tick is taken care of by 2's complement arithmetic.
          if (millis() - start >= timeout) {
              // timeout
              if( is_reopen == FALSE )
              {
                is_opened = FALSE;
              }
              usb_cdc_debug_cnt[1]++;
              return;
          }
          __WFI(); // enter sleep mode, waiting for interrupt
      }

      // Write data to device buffer
      UserTxBuffer[UserTxBufPtrIn] = p_buf[i];
      UserTxBufPtrIn = (UserTxBufPtrIn + 1) & (APP_TX_DATA_SIZE - 1);
  }
}
#endif

/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_TxAvailable
     WORK    :
---------------------------------------------------------------------------*/
uint32_t CDC_Itf_TxAvailable( void )
{
  uint32_t length = 0;

  __disable_irq();
  length = (APP_TX_DATA_SIZE + UserTxBufPtrIn - UserTxBufPtrOut) % APP_TX_DATA_SIZE;
  length = APP_TX_DATA_SIZE - length;
  __enable_irq();

  return length;
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
     TITLE   : CDC_Itf_Available
     WORK    :
---------------------------------------------------------------------------*/
uint32_t CDC_Itf_Available( void )
{
  uint32_t length;

  __disable_irq();
  length = (APP_RX_BUF_SIZE + rxd_BufPtrIn - rxd_BufPtrOut) % APP_RX_BUF_SIZE;
  __enable_irq();

  return length;
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_Peek
     WORK    :
---------------------------------------------------------------------------*/
int32_t CDC_Itf_Peek( void )
{

  if( rxd_BufPtrIn == rxd_BufPtrOut ) return -1;


  return rxd_buffer[rxd_BufPtrOut];
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_IsConnected
     WORK    :
---------------------------------------------------------------------------*/
BOOL CDC_Itf_IsConnected( void )
{
  if( USBD_Device.dev_config == 0
    || is_opened == FALSE
    || USBD_Device.pClassData == NULL )
  {
    return FALSE;
  }

  return TRUE;
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


BOOL CDC_Itf_IsTxTransmitted( void )
{
  return (UserTxBufPtrIn == UserTxBufPtrOut) ? TRUE : FALSE;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
