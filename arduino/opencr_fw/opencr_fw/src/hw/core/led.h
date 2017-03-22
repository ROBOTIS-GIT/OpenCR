/*
 *  led.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef LED_H
#define LED_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"



#define LED_CH_MAX      4





void ledInit();


void ledOn(uint8_t ch);
void ledOff(uint8_t ch);
void ledToggle(uint8_t ch);


#ifdef __cplusplus
}
#endif


#endif
