/*
 * can.h
 *
 *  Created on: 2017. 12. 5.
 *      Author: opus
 */

#ifndef CAN_H_
#define CAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <chip.h>




typedef struct {
  uint32_t id;
  uint32_t length;
  uint8_t  data[8];
} can_msg_t;


void canInit(void);
bool canOpen(uint8_t channel, uint32_t baudrate, uint8_t format);
void canClose(uint8_t channel);
bool canConfigFilter(uint8_t filter_num, uint32_t id, uint32_t mask);
uint32_t canWrite(uint8_t channel, uint32_t id, uint8_t *data, uint32_t length);
uint8_t canRead(uint8_t channel);
uint32_t canAvailable(uint8_t channel);
uint32_t canWriteMsg(uint8_t channel, can_msg_t *p_msg);
bool canReadMsg(uint8_t channel, can_msg_t *p_msg);
uint32_t canAvailableMsg(uint8_t channel);

uint8_t canGetErrCount(uint8_t channel);
uint32_t canGetError(uint8_t channel);
uint32_t canGetState(uint8_t channel);

void canAttachRxInterrupt(uint8_t channel, void (*handler)(can_msg_t *arg));
void canDetachRxInterrupt(uint8_t channel);




#ifdef __cplusplus
 }
#endif


#endif /* CAN_H_ */
