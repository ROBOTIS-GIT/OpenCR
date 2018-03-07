/*
 *  msg.c
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "msg.h"
#include "vcp.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>



void msg_init(void)
{

}


void msg_send(uint8_t ch, mavlink_message_t *p_msg)
{
  uint8_t  buf[1024];
  uint32_t len;
  int i;


  len = mavlink_msg_to_send_buffer(buf, p_msg);

  switch(ch)
  {
    case 0:
      vcp_write(buf, len);
      break;

    case 1:
      break;
  }
}


BOOL msg_recv( uint8_t ch, uint8_t data , msg_t *p_msg )
{
  BOOL ret = FALSE;
  static mavlink_message_t msg[MSG_CH_MAX];
  mavlink_status_t status[MSG_CH_MAX];


  p_msg->ch = ch;

  if(ch == 0)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, data, &msg[ch], &status[ch]) == MAVLINK_FRAMING_OK)
    {
      p_msg->p_msg = &msg[ch];
      ret = TRUE;
    }
  }
  else
  {
    if (mavlink_parse_char(MAVLINK_COMM_1, data, &msg[ch], &status[ch]) == MAVLINK_FRAMING_OK)
    {
      p_msg->p_msg = &msg[ch];
      ret = TRUE;
    }
  }
  return ret;
}
