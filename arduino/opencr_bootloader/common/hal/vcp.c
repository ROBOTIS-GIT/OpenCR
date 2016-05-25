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
#include "usbd_cdc_interface.h"



void vcp_init(void)
{

}


BOOL vcp_is_available(void)
{
  BOOL ret = FALSE;

  // TODO : 시리얼 버퍼에 데이터가 있으면 TRUE 리턴한다.

  ret = CDC_Itf_IsAvailable();

  return ret;
}


void vcp_putch(uint8_t ch)
{
  // TODO : 시리얼포트로 1바이트 데이터 전송
  CDC_Itf_Write( &ch, 1 );
}


uint8_t vcp_getch(void)
{
  // TODO : 시리얼포트로 부터 1바이트 데이터 수신 (Block 방식)
  return CDC_Itf_Getch();
}


int32_t vcp_write(uint8_t *p_data, uint32_t length)
{
  // TODO : 시리얼 포트로 length 길이만큼의 문자열을 전송함

  CDC_Itf_Write( p_data, length );
  return length;
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


  ret = vcp_write( print_buffer, len);

  return ret;
}

