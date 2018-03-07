/*
 *  msg.h
 *
 *  message process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef MSG_H
#define MSG_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"



#define MSG_CH_MAX	1


typedef struct
{
  uint8_t ch;
  mavlink_message_t *p_msg;
} msg_t;



void msg_init(void);
void msg_send(uint8_t chan, mavlink_message_t *p_msg);
BOOL msg_recv( uint8_t chan, uint8_t data , mavlink_message_t *p_msg, mavlink_status_t *p_status );
BOOL msg_get_resp( uint8_t chan, mavlink_message_t *p_msg, uint32_t timeout);


#ifdef __cplusplus
}
#endif


#endif

