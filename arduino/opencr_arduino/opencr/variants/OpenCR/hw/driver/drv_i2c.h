/*
 *  drv_i2c.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_I2C_H
#define DRV_I2C_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "def.h"
#include "bsp.h"




#define DRV_I2C_CNT 2

extern I2C_HandleTypeDef drv_i2c_handles[DRV_I2C_CNT];     // If we are doing hardware I2C
 

int drv_i2c_init();



#ifdef __cplusplus
}
#endif


#endif
