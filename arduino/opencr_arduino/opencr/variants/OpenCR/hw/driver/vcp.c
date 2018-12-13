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
#include "usbd_cdc_interface.h"


extern uint32_t usb_cdc_debug_cnt[];


void vcp_init(void)
{

}


uint32_t vcp_is_available(void)
{
  return CDC_Itf_Available();
}


int32_t vcp_peek(void)
{
  return CDC_Itf_Peek();
}


BOOL vcp_is_connected(void)
{
  return CDC_Itf_IsConnected();
}


void vcp_putch(uint8_t ch)
{
  CDC_Itf_Write( &ch, 1 );
}


uint8_t vcp_getch(void)
{
#if 1
  return CDC_Itf_Getch();
#else
  /* for debugging */
  uint8_t ch = CDC_Itf_Getch();
  drv_uart_write(DRV_UART_NUM_4, ch);
  return ch;
#endif
}


int32_t vcp_write(uint8_t *p_data, uint32_t length)
{
  int32_t  ret;

  #if 1
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
    if(ret == (int32_t) length)
    {
      break;
    }
    if(millis()-t_time > 100)
    {
      usb_cdc_debug_cnt[1]++;
      ret = 0;
      break;
    }
  }
  #else
  ret = CDC_Itf_Write( p_data, length );


  if(ret < 0)
  {
    ret = 0;
  }
  #endif
  return ret;
}


int32_t vcp_printf( const char *fmt, ...)
{
  int32_t ret = 0;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  static char print_buffer[255];

  len = vsnprintf(print_buffer, 255, fmt, arg);
  va_end (arg);


  ret = vcp_write( (uint8_t *) print_buffer, len);

  return ret;
}


BOOL vcp_is_transmitted( void )
{
  return CDC_Itf_IsTxTransmitted();
}