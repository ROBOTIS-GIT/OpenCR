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

#ifndef _VARIANT_NUCLEO_F1xx_
#define _VARIANT_NUCLEO_F1xx_

#include <chip.h>


/** Master clock frequency */
#define VARIANT_MCK			F_CPU

#define NO_ADC 0xffff

#define NO_PWM 0xffff
#define hTimer1 1
#define hTimer2 2
#define hTimer3 3
#define hTimer4 4


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#endif


#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

#ifdef USE_USART1
extern UART_HandleTypeDef huart1;
#endif

#ifdef USE_USART2
extern UART_HandleTypeDef huart2;
#endif

#ifdef USE_USART3
extern UART_HandleTypeDef huart3;
#endif

#ifdef USE_USBSerial
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

/*
	From Roger Clark's maple port.  Convenance alias to
	STM Nucleo abstractions.  If the LED is not blinking
	check this that this mapping is correct for the
	Device Under Test (DIT) These arrays must work
	together and are the main reason we have so many STM32
	Variants
 
 Pin aliases: these give the GPIO port/bit for each pin as an
 * enum. These are optional, but recommended. They make it easier to
 * write code using low-level GPIO functionality. 
 
*/
/* This mapping is for from Blue pill Schematic */
enum {
  PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13,	PA14, PA15,
	PB0, PB1, PB2, PB3, PB4, PB5,	PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
  PC13, PC14, PC15
};

/*
 * Analog pins
 */
static const uint8_t A0  = PA0;
static const uint8_t A1  = PA1;
static const uint8_t A2  = PA2;
static const uint8_t A3  = PA3;
static const uint8_t A4  = PA4;
static const uint8_t A5  = PA5;
static const uint8_t A6  = PA6;
static const uint8_t A7  = PA7;
static const uint8_t A8  = PB0;
static const uint8_t A9  = PB1;

/* Definitions and types for pins */

/* low level defines are declared in STM32CubeMX headers */

/* 
	This structure maps the high level settings of the port 
	See the HALMX core function variant and wiring_digital 
	to see how to use 
*/
typedef struct _Pin2PortMapArray
{
	/* 
		define the Abstract Hal port reference
		
		IMPORTANT NOTE: this definition is opaque and
		defined inside the chip family
		do not try and change the include name locally
		unless you know what you are doing
		
	*/	
  	GPIO_TypeDef *GPIOx_Port; 		/* defined in stm32f401xe.h */
  	/* 
  		we only record the pin here as the typedef record
  		is normally mutable and only used by the init code
  	*/
  	uint32_t 	Pin_abstraction;	/* must match type in GPIO_InitTypeDef struct */

    uint32_t  adc_channel;
    uint32_t  timerNumber;   //Timer1 to Timer4.
    uint32_t  timerChannel;  //Timer channel (1-4).
} Pin2PortMapArray ;

/* Pins table to be instanciated into variant.cpp */
extern const Pin2PortMapArray g_Pin2PortMapArray[] ;
//void UART_Handler(void);
void Rx1_Handler(void); /* Vassilis Serasidis */
void Tx1_Handler(void); /* Vassilis Serasidis */
void Rx2_Handler(void); /* Vassilis Serasidis */
void Tx2_Handler(void); /* Vassilis Serasidis */
void Rx3_Handler(void); /* Vassilis Serasidis */
void Tx3_Handler(void); /* Vassilis Serasidis */
void USBSerial_Rx_Handler(uint8_t *data, uint16_t len); /* Vassilis Serasidis */
void USBSerial_Tx_Handler(uint8_t *data, uint16_t len); /* Vassilis Serasidis */
void StartUSBSerial(void);
#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

#ifdef USE_USART1
extern UARTClass Serial1;
#endif

#ifdef USE_USART2
extern UARTClass Serial2;
#endif

#ifdef USE_USART3
extern UARTClass Serial3;
#endif

#ifdef USE_USBSerial
extern USBSerial Serial;
#endif

#endif

#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA         (PB7)
#define PIN_WIRE_SCL         (PB8)
#define WIRE_INTERFACE       hi2c1
#define WIRE_INTERFACE_ID    I2C2

#define PIN_WIRE1_SDA        (PB11)
#define PIN_WIRE1_SCL        (PB10)
#define WIRE1_INTERFACE      hi2c2
#define WIRE1_INTERFACE_ID   I2C2


#define SPI_INTERFACES_COUNT 2

#endif
