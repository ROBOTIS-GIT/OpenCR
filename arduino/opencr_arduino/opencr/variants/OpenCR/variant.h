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



typedef struct _Pin2PortMapArray
{
  	GPIO_TypeDef *GPIOx_Port;

  	uint32_t 	Pin_abstraction;

  	ADC_HandleTypeDef *ADCx;
    uint32_t  adc_channel;

    TIM_HandleTypeDef *TIMx;
    uint32_t  timerChannel;
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

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

#define WIRE_INTERFACES_COUNT       1
#define PIN_WIRE_SDA                (PB7)
#define PIN_WIRE_SCL                (PB8)
#define WIRE_INTERFACE              hi2c1
#define WIRE_INTERFACE_ID           I2C2

#define PIN_WIRE1_SDA               (PB11)
#define PIN_WIRE1_SCL               (PB10)
#define WIRE1_INTERFACE             hi2c2
#define WIRE1_INTERFACE_ID          I2C2


#define SPI_INTERFACES_COUNT        2

#endif
