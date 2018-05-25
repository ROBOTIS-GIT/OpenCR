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


void     vcp_init(void);
uint32_t vcp_is_available(void);
int32_t  vcp_peek(void);
BOOL     vcp_is_connected(void);
void     vcp_putch(uint8_t ch);
uint8_t  vcp_getch(void);
int32_t  vcp_write(uint8_t *p_data, uint32_t length);

int32_t  vcp_printf( const char *fmt, ...);

BOOL     vcp_is_transmitted(void);

#ifdef __cplusplus
}
#endif


#endif

