/*
 *  button.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef BUTTON_H
#define BUTTON_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"







void button_init(void);

uint8_t button_read( uint8_t pin_num );


#ifdef __cplusplus
}
#endif


#endif

