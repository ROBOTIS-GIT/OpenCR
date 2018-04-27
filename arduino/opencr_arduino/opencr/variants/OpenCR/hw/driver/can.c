/*
 * can.c
 *
 *  Created on: 2017. 12. 5.
 *      Author: opus
 */

#include <string.h>
#include "can.h"
#include "drv_can.h"




void canInit(void)
{
  drvCanInit();
}


bool canOpen(uint32_t baudrate, uint8_t format)
{
  return drvCanOpen(_DEF_CAN2, baudrate, format);
}

void canClose(void)
{
  drvCanClose(_DEF_CAN2);
}

bool canConfigFilter(uint32_t id, uint32_t mask)
{
  return drvCanConfigFilter(14, id, mask);
}

uint32_t canWrite(uint32_t id, uint8_t *p_data, uint32_t length)
{
  return drvCanWrite(_DEF_CAN2, id, p_data, length);
}

uint8_t canRead(void)
{
  return drvCanRead(_DEF_CAN2);
}

uint32_t canAvailable(void)
{
  return drvCanAvailable(_DEF_CAN2);
}

uint32_t canWriteMsg(can_msg_t *p_msg)
{
  drv_can_msg_t msg;
  msg.id = p_msg->id;
  msg.length = p_msg->length;
  memcpy(msg.data, p_msg->data, 8);

  return drvCanWriteMsg(_DEF_CAN2, &msg);
}

bool canReadMsg(can_msg_t *p_msg)
{
  bool ret = false;
  drv_can_msg_t *rx_msg;

  rx_msg = drvCanReadMsg(_DEF_CAN2);

  if(rx_msg != NULL)
  {
    p_msg->id = rx_msg->id;
    p_msg->length = rx_msg->length;

    memcpy(p_msg->data, rx_msg->data, p_msg->length);

    ret = true;
  }

  return ret;
}

uint32_t canAvailableMsg(void)
{
  return drvCanAvailableMsg(_DEF_CAN2);
}

uint8_t canGetErrCount(void)
{
  return drvCanGetErrCount(_DEF_CAN2);
}

uint32_t canGetError(void)
{
  return drvCanGetError(_DEF_CAN2);
}

uint32_t canGetState(void)
{
  return drvCanGetState(_DEF_CAN2);
}

void canAttachRxInterrupt(void (*handler)(can_msg_t *arg))
{
  drvCanAttachRxInterrupt(_DEF_CAN2, (void(*)(void *arg)) handler);
}

void canDetachRxInterrupt(void)
{
  drvCanDetachRxInterrupt(_DEF_CAN2);
}



