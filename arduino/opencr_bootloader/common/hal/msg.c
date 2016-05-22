/*
 *  msg.c
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
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
  uint8_t  buf2[1024];
  uint32_t len;
  int i;


  len = mavlink_msg_to_send_buffer(buf, p_msg);

  switch(ch)
  {
    case 0:
      //vcp_printf("msg_send %d \r\n", len);
      //vcp_printf("msg_send %d \r\n", len);
      //vcp_write("aaaaaaaaa", 6);
      for( i=0; i<len; i++ )
      buf[i] = i;
      /*
      buf[0]=254;
      buf[1]=18;
      buf[2]=1;
      buf[3]=0;
      buf[4]=0;
      buf[5]	=	150;
      buf[6]	=	1 ;
      buf[7]	=	9 ;
      buf[8]	=	86 ;
      buf[9]	=	49 ;
      buf[10]	=	54 ;
      buf[11]	=	48 ;
      buf[12]	=	53 ;
      buf[13]	=	50 ;
      buf[14]	=	49 ;
      buf[15]	=	82 ;
      buf[16]	=	49 ;
      buf[17]	=	0 ;
      buf[18]	=	0 ;
      buf[19]	=	0 ;
      buf[20]	=	79 ;
      buf[21]	=	112 ;
      buf[22]	=	101 ;
      buf[23]	=	110 ;
      buf[24]	=	68 ;
      buf[25]	=	110 ;
    */

      vcp_write(buf, len);
      break;

    case 1:
      break;
  }
}


BOOL msg_recv( uint8_t ch, uint8_t data , msg_t *p_msg )
{
  BOOL ret = FALSE;
  mavlink_message_t msg[MSG_CH_MAX];
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
