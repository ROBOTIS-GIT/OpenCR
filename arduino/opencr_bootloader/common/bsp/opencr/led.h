/*
 *  led.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#ifndef LED_H
#define LED_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"


void led_init(void);
void led_on(uint8_t ch);
void led_off(uint8_t ch);
void led_toggle(uint8_t ch);


#ifdef __cplusplus
}
#endif


#endif

