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

static uint8_t  rxd_buffer[APP_RX_BUF_SIZE];
static uint32_t rxd_length    = 0;
static uint32_t rxd_BufPtrIn  = 0;
static uint32_t rxd_BufPtrOut = 0;


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
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
  is_opened = FALSE;
  LineCoding.bitrate = 0;
  usb_cdc_bitrate = 0;

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


// This function is called to process outgoing data.  We hook directly into the
// SOF (start of frame) callback so that it is called exactly at the time it is
// needed (reducing latency), and often enough (increasing bandwidth).
//
// this is based on micropyton
//   : https://github.com/micropython/micropython/blob/master/stmhal/usbd_cdc_interface.c
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
    if (is_opened == FALSE) {
        // CDC device is not connected to a host, so we are unable to send any data
        return;
    }

    if (UserTxBufPtrOut == UserTxBufPtrIn && !UserTxNeedEmptyPacket) {
        // No outstanding data to send
        return;
    }

    if (UserTxBufPtrOut != UserTxBufPtrOutShadow) {
        // We have sent data and are waiting for the low-level USB driver to
        // finish sending it over the USB in-endpoint.
        // SOF occurs every 1ms, so we have a 150 * 1ms = 150ms timeout
        if (UserTxBufPtrWaitCount < 150) {
            USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
            if (USBx_INEP(CDC_IN_EP & 0x7f)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) {
                // USB in-endpoint is still reading the data
                UserTxBufPtrWaitCount++;
                return;
            }
        }
        UserTxBufPtrOut = UserTxBufPtrOutShadow;
    }

    if (UserTxBufPtrOutShadow != UserTxBufPtrIn || UserTxNeedEmptyPacket) {
        uint32_t buffptr;
        uint32_t buffsize;

        if (UserTxBufPtrOutShadow > UserTxBufPtrIn) { // rollback
            buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOutShadow;
        } else {
            buffsize = UserTxBufPtrIn - UserTxBufPtrOutShadow;
        }

        buffptr = UserTxBufPtrOutShadow;

        USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[buffptr], buffsize);

        if (USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK) {
            UserTxBufPtrOutShadow += buffsize;
            if (UserTxBufPtrOutShadow == APP_TX_DATA_SIZE) {
                UserTxBufPtrOutShadow = 0;
            }
            UserTxBufPtrWaitCount = 0;

            // According to the USB specification, a packet size of 64 bytes (CDC_DATA_FS_MAX_PACKET_SIZE)
            // gets held at the USB host until the next packet is sent.  This is because a
            // packet of maximum size is considered to be part of a longer chunk of data, and
            // the host waits for all data to arrive (ie, waits for a packet < max packet size).
            // To flush a packet of exactly max packet size, we need to send a zero-size packet.
            // See eg http://www.cypress.com/?id=4&rID=92719
            UserTxNeedEmptyPacket = (buffsize > 0 && buffsize % CDC_DATA_FS_MAX_PACKET_SIZE == 0 && UserTxBufPtrOutShadow == UserTxBufPtrIn);
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

  USBD_CDC_ReceivePacket(&USBD_Device);
  return (USBD_OK);
}


/*---------------------------------------------------------------------------
     TITLE   : CDC_Itf_Write
     WORK    :
---------------------------------------------------------------------------*/
void CDC_Itf_Write( uint8_t *p_buf, uint32_t length )
{
  uint32_t timeout = 100;


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
              return;
          }
          __WFI(); // enter sleep mode, waiting for interrupt
      }

      // Write data to device buffer
      UserTxBuffer[UserTxBufPtrIn] = p_buf[i];
      UserTxBufPtrIn = (UserTxBufPtrIn + 1) & (APP_TX_DATA_SIZE - 1);
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
     TITLE   : CDC_Itf_Available
     WORK    :
---------------------------------------------------------------------------*/
uint32_t CDC_Itf_Available( void )
{
  uint32_t length;

  __disable_irq();
  length = (rxd_BufPtrIn - rxd_BufPtrOut) % APP_RX_BUF_SIZE;
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
  if( USBD_Device.dev_config == 0 || is_opened == FALSE ) return FALSE;

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




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
