/*
 *  wdg.h
 *
 *  Created on: 2016. 7. 8.
 *      Author: Baram, PBHP
 */

#ifndef WDG_H
#define WDG_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


void wdg_init(void);
BOOL wdg_setup(uint32_t reload_time);
BOOL wdg_start(void);
BOOL wdg_refresh(void);
BOOL wdg_get_reset(void);

#ifdef __cplusplus
}
#endif


#endif
