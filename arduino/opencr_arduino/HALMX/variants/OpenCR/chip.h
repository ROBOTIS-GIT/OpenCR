/* stub for hardware types */
#ifndef _CHIP_OPENCR_F7xx_
#define _CHIP_OPENCR_F7xx_

#include <stdbool.h>  /* for wiring constants */

#include "hal.h"  /* Ideally this is defined in variant */


/* define some abstractions that are identical to Arduino */

#define millis(a1) 	HAL_GetTick(a1)
#define delay(a2) 	HAL_Delay(a2)




/**
 *     12 April 2016 by Vassilis Serasidis
 */
//Comment out the Serial port you need to use.


#define USE_USART1
#define USE_USART2
//#define USE_USART3

//Comment out the SPI port you need to use.
#define USE_SPI1
#define USE_SPI2
/********************************************************/

#define BOARD_NR_I2C 2
#define HAL_I2C1 I2C1
#define HAL_I2C2 I2C2
#define HAL_I2C3 I2C3

//Number of Analog-to-Digital-Converter (ADC) pins that has the STM32F103C8 MCU
#define BOARD_NR_ADC_PINS 5 //The STM32F103C8 has 9 Analog inputs.

//Number of Pulse-Width-Modulation (PWM) pins that has the STM32F103C8 MCU
#define BOARD_NR_PWM_PINS 12

//Include this line if the disconnect pin is used (The maple mini uses the DISC pin PB9).
#define USB_DISC_PIN PB9

#define HAL_GPIOB GPIOB




#endif
