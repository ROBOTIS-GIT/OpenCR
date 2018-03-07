/*
 *  msg.h
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef MSG_H
#define MSG_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


#define MSG_CH_MAX	1


typedef struct
{
  uint8_t ch;
  mavlink_message_t *p_msg;
} msg_t;



void msg_init(void);
void msg_send(uint8_t ch, mavlink_message_t *p_msg);
BOOL msg_recv( uint8_t ch, uint8_t data , msg_t *p_msg );



#ifdef __cplusplus
}
#endif


#endif

