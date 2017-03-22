/*
 *  micros.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef MICROS_H
#define MICROS_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "def.h"
#include "bsp.h"






void microsInit();

uint32_t micros();




#ifdef __cplusplus
}
#endif


#endif
