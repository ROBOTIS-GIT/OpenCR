/*
 *  dxl_hw.h
 *
 *  dynamixel hardware
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#ifndef DXL_HW_H
#define DXL_HW_H


#include "dxl_def.h"


#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif


#define DXL_LED_RX            BDPIN_LED_USER_1
#define DXL_LED_TX            BDPIN_LED_USER_2


uint32_t dxl_hw_begin(uint8_t baud);

void dxl_hw_tx_enable(void);
void dxl_hw_tx_disable(void);

void dxl_hw_power_enable(void);
void dxl_hw_power_disable(void);

uint8_t dxl_hw_read(void);
void    dxl_hw_write(uint8_t value);
void    dxl_hw_write(uint8_t *p_data, uint32_t length);

uint32_t dxl_hw_available(void);

#endif
