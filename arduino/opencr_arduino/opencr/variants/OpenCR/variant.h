/****************************************************************************
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 *
 * Variant definition library for Arduino STM32 + HAL + CubeMX (HALMX).
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 ****************************************************************************/
 /*
 *  Modified on: 2016. 7.12.
 *       Author: Baram, PBHP
 */
#ifndef _VARIANT_OPENCR_
#define _VARIANT_OPENCR_

#include <chip.h>



#define NO_ADC 		0xffff
#define NO_PWM		0xffff
#define NO_EXTI   0xffff


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USBSerial.h"
#endif


#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


 

/*
 * Analog pins
 */
static const uint8_t A0  = 16;
static const uint8_t A1  = 17;
static const uint8_t A2  = 18;
static const uint8_t A3  = 19;
static const uint8_t A4  = 20;
static const uint8_t A5  = 21;
static const uint8_t BAT = 29;



typedef struct _Pin2PortMapArray
{
  	GPIO_TypeDef *GPIOx_Port;

  	uint32_t 	Pin_abstraction;

  	ADC_HandleTypeDef *ADCx;
    uint32_t  adc_channel;

    TIM_HandleTypeDef *TIMx;
    uint32_t  timerChannel;
    uint32_t  extiChannel;
} Pin2PortMapArray ;


extern const Pin2PortMapArray g_Pin2PortMapArray[] ;

void Rx1_Handler(void);
void Tx1_Handler(void);
void Rx2_Handler(void);
void Tx2_Handler(void);

#ifdef __cplusplus
}
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern USBSerial Serial;
extern UARTClass Serial1;
extern UARTClass Serial2;



#endif


#define digitalPinToInterrupt(P)   ( g_Pin2PortMapArray[P].extiChannel )


#define WIRE_INTERFACES_COUNT       1
#define SPI_INTERFACES_COUNT        2
#define EXTI_COUNT                  5
#define PINS_COUNT                  64

#endif
