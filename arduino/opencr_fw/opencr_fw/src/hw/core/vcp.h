/*
 *  vcp.h
 *
 *  virtual_com_port
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef VCP_H
#define VCP_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


void     vcpInit(void);
uint32_t vcpAvailable(void);
int32_t  vcpPeek(void);
bool     vcpIsConnected(void);
void     vcpPutch(uint8_t ch);
uint8_t  vcpGetch(void);
uint8_t  vcpRead(void);
int32_t  vcpWrite(uint8_t *p_data, uint32_t length);

int32_t  vcpPrintf( const char *fmt, ...);


#ifdef __cplusplus
}
#endif


#endif

