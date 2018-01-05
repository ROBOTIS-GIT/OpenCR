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
bool canOpen(uint32_t baudrate, uint8_t format);
void canClose(void);
bool canConfigFilter(uint32_t id, uint32_t mask);
uint32_t canWrite(uint32_t id, uint8_t *data, uint32_t length);
uint8_t canRead(void);
uint32_t canAvailable(void);
uint32_t canWriteMsg(can_msg_t *p_msg);
bool canReadMsg(can_msg_t *p_msg);
uint32_t canAvailableMsg(void);

uint8_t canGetErrCount(void);
uint32_t canGetError(void);
uint32_t canGetState(void);

void canAttachRxInterrupt(void (*handler)(can_msg_t *arg));
void canDetachRxInterrupt(void);




#ifdef __cplusplus
 }
#endif


#endif /* CAN_H_ */
