#ifndef _CHIP_OPENCR_F7xx_
#define _CHIP_OPENCR_F7xx_

#include <stdbool.h>
#include "hal.h"
#include "drv.h"



#define millis(a1) 	HAL_GetTick(a1)
#define delay(a2) 	HAL_Delay(a2)



#define USE_SPI1
#define USE_SPI2


#define BOARD_NR_I2C 2
#define HAL_I2C1 I2C1
#define HAL_I2C2 I2C2
#define HAL_I2C3 I2C3


#define BOARD_NR_ADC_PINS   5
#define BOARD_NR_PWM_PINS   12


#define BDPIN_SPI_CS_IMU    28


#endif
