/*
 *  drv_eeprom.h
 *
 *  Created on: 2016. 10.06.
 *      Author: Baram, PBHP
 */

#ifndef DRV_EEPROM_H
#define DRV_EEPROM_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"








int drv_eeprom_init();

uint8_t  drv_eeprom_read_byte(int addr);
void     drv_eeprom_write_byte(int index, uint8_t data_in);
uint16_t drv_eeprom_get_length(void);


#ifdef __cplusplus
}
#endif


#endif
