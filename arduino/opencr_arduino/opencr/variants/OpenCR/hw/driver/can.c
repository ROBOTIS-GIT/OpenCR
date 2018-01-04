/*
 * can.c
 *
 *  Created on: 2017. 12. 5.
 *      Author: opus
 */

#include "can.h"
#include "drv_can.h"




void canInit(void)
{
  drvCanInit();
}


bool canOpen(uint8_t channel, uint32_t baudrate, uint8_t format)
{
  return drvCanOpen(channel, baudrate, format);
}

void canClose(uint8_t channel)
{
  drvCanClose(channel);
}

bool canConfigFilter(uint8_t filter_num, uint32_t id, uint32_t mask)
{
  return drvCanConfigFilter(filter_num, id, mask);
}

uint32_t canWrite(uint8_t channel, uint32_t id, uint8_t *data, uint32_t length)
{
  return drvCanWrite(channel, id, data, length);
}

uint8_t canRead(uint8_t channel)
{
  return drvCanRead(channel);
}

uint32_t canAvailable(uint8_t channel)
{
  return drvCanAvailable(channel);
}

uint32_t canWriteMsg(uint8_t channel, can_msg_t *p_msg)
{
  drv_can_msg_t msg;
  msg.id = p_msg->id;
  msg.length = p_msg->length;
  memcpy(msg.data, p_msg->data, 8);

  return drvCanWriteMsg(channel, &msg);
}

bool canReadMsg(uint8_t channel, can_msg_t *p_msg)
{
  bool ret = false;
  drv_can_msg_t *rx_msg;

  rx_msg = drvCanReadMsg(channel);

  if(rx_msg != NULL)
  {
    p_msg->id = rx_msg->id;
    p_msg->length = rx_msg->length;

    memcpy(p_msg->data, rx_msg->data, p_msg->length);

    ret = true;
  }

  return ret;
}

uint32_t canAvailableMsg(uint8_t channel)
{
  return drvCanAvailableMsg(channel);
}

uint8_t canGetErrCount(uint8_t channel)
{
  return drvCanGetErrCount(channel);
}

uint32_t canGetError(uint8_t channel)
{
  return drvCanGetError(channel);
}

uint32_t canGetState(uint8_t channel)
{
  return drvCanGetState(channel);
}

void canAttachRxInterrupt(uint8_t channel, void (*handler)(can_msg_t *arg))
{
  drvCanAttachRxInterrupt(channel, (void(*)(void *arg)) handler);
}

void canDetachRxInterrupt(uint8_t channel)
{
  drvCanDetachRxInterrupt(channel);
}



