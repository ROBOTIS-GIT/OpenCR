/*
 *  cmd.h
 *
 *  command process
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef CMD_H
#define CMD_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"
#include "hal.h"





void cmd_init(void);

void cmd_version( msg_t *p_msg );


#ifdef __cplusplus
}
#endif


#endif

