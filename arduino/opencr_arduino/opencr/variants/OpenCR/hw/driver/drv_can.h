/*
 * drv_can.h
 *
 *  Created on: 2017. 11. 7.
 *      Author: opus
 */

#ifndef DRV_CAN_H_
#define DRV_CAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "def.h"
#include "bsp.h"



#define DRV_CAN_MAX_CH           1
#define DRV_CAN_MAX_BYTE_IN_MSG  8
#define DRV_CAN_MSG_RX_BUF_MAX   8
#define DRV_CAN_DATA_RX_BUF_MAX  128

typedef struct {
  uint32_t id;
  uint32_t length;
  uint8_t  data[DRV_CAN_MAX_BYTE_IN_MSG];
  uint8_t  format;
} drv_can_msg_t;


void drvCanInit(void);
bool drvCanOpen(uint8_t channel, uint32_t baudrate, uint8_t format);
void drvCanClose(uint8_t channel);
bool drvCanConfigFilter(uint8_t filter_num, uint32_t id, uint32_t mask, uint8_t format);
uint32_t drvCanWrite(uint8_t channel, uint32_t id, uint8_t *p_data, uint32_t length, uint8_t format);
uint8_t drvCanRead(uint8_t channel);
uint32_t drvCanAvailable(uint8_t channel);
uint32_t drvCanWriteMsg(uint8_t channel, drv_can_msg_t *p_msg);
drv_can_msg_t* drvCanReadMsg(uint8_t channel);
uint32_t drvCanAvailableMsg(uint8_t channel);

uint8_t drvCanGetErrCount(uint8_t channel);
uint32_t drvCanGetError(uint8_t channel);
uint32_t drvCanGetState(uint8_t channel);

void drvCanAttachRxInterrupt(uint8_t channel, void (*handler)(void *arg));
void drvCanDetachRxInterrupt(uint8_t channel);


#ifdef __cplusplus
 }
#endif

#endif /* DRV_CAN_H_ */
