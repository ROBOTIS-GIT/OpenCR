/*
 *  vcp.c
 *
 *  virtual_com_port
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "vcp.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "hw.h"
#include "usb_cdc/usbd_cdc_interface.h"


extern uint32_t usb_cdc_debug_cnt[];


void vcpInit(void)
{

}

uint32_t vcpAvailable(void)
{
  return CDC_Itf_Available();
}

int32_t vcpPeek(void)
{
  return CDC_Itf_Peek();
}

bool vcpIsConnected(void)
{
  return CDC_Itf_IsConnected();
}

void vcpPutch(uint8_t ch)
{
  CDC_Itf_Write( &ch, 1 );
}

uint8_t vcpGetch(void)
{
  return CDC_Itf_Getch();
}

uint8_t vcpRead(void)
{
  uint8_t ret;

  ret = CDC_Itf_Getch();

  return ret;
}

int32_t vcpWrite(uint8_t *p_data, uint32_t length)
{
  int32_t  ret;
  uint32_t t_time;

  t_time = millis();
  while(1)
  {
    ret = CDC_Itf_Write( p_data, length );

    if(ret < 0)
    {
      ret = 0;
      break;
    }
    if(ret == length)
    {
      break;
    }
    if(millis()-t_time > 100)
    {
      ret = 0;
      break;
    }
  }

  return ret;
}

int32_t vcpPrintf( const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  static char print_buffer[255];

  len = vsnprintf(print_buffer, 255, fmt, arg);
  va_end (arg);


  ret = vcpWrite( print_buffer, len);

  return ret;
}
