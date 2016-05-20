/*
 *  flash.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#ifndef FLASH_H
#define FLASH_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"



#define FLASH_OK              0
#define FLASH_ERR_TIMEOUT     1





void flash_init(void);

uint8_t flash_write(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout );
uint8_t flash_read(uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout );


#ifdef __cplusplus
}
#endif


#endif

