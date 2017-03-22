/*
 *  millis.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef MILLIS_H
#define MILLIS_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"





void millisInit();

uint32_t millis();



#ifdef __cplusplus
}
#endif


#endif
