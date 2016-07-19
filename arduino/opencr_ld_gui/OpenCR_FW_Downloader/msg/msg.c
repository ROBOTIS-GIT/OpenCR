/*
 *  msg.c
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#include "msg.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
//#include "../serial.h"


extern ser_handler stm32_ser_id;
extern int read_byte( void );
extern int write_bytes( char *p_data, int len );

void msg_init(void)
{

}


void msg_send(uint8_t chan, mavlink_message_t *p_msg)
{
  uint8_t  buf[1024];
  uint16_t len;
  uint16_t write_len;

  len = mavlink_msg_to_send_buffer(buf, p_msg);

  switch(chan)
  {
    case 0:
      write_len = write_bytes((char *)buf, (uint32_t)len);
      if( write_len != len ) printf("wlen %d : len %d\r\n", write_len, len);
      break;

    case 1:
      break;
  }
}


BOOL msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status )
{
  BOOL ret = FALSE;

if (mavlink_parse_char(MAVLINK_COMM_0, data, p_msg, p_status) == MAVLINK_FRAMING_OK)
{
  ret = TRUE;
}

  return ret;
}


BOOL msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout)
{
  BOOL ret = FALSE;
  int  ch_ret;
  uint8_t ch;
  static mavlink_message_t msg[MSG_CH_MAX];
  static mavlink_status_t status[MSG_CH_MAX];
  uint32_t retry = timeout;



  ser_set_timeout_ms( stm32_ser_id, 1 );

  while(1)
  {
    ch_ret = read_byte();

    if( ch_ret < 0 )
    {
      if( retry-- <= 0 )
      {
	ret = FALSE;
	break;
      }
      else
      {
	continue;
      }
    }
    else
    {
      ch = (uint8_t)(ch_ret);
      retry = timeout;
    }

    ret = msg_recv( chan, ch, &msg[chan], &status[chan] );

    if( ret == TRUE )
    {
      *p_msg = msg[chan];
      break;
    }
  }

  return ret;
}

