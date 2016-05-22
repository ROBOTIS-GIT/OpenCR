/*
 *  cmd.c
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#include "cmd.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>



void cmd_init(void)
{

}


void cmd_version( msg_t *p_msg )
{
  mavlink_message_t mav_msg;
  uint8_t buf[1024];


  mavlink_msg_version_pack(0, 0, &mav_msg, 1, 9, (const uint8_t*) "V160521R1");

  msg_send(p_msg->ch, &mav_msg);
}
