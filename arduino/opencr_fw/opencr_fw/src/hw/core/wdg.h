/*
 *  wdg.h
 *
 *  Created on: 2016. 7. 8.
 *      Author: Baram
 */

#ifndef WDG_H
#define WDG_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


void wdgInit(void);
bool wdgSetup(uint32_t reload_time);
bool wdgStart(void);
bool wdgGetReset(void);

#ifdef __cplusplus
}
#endif


#endif
