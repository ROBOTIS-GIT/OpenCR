/****************************************Copyright (c)**************************************************
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SCCB.h
** Descriptions:		SCCB ����������
**
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-2-13
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
********************************************************************************************************/
#ifndef __SCCB_H
#define __SCCB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"
#include <hw.h>

/* Private define ------------------------------------------------------------*/
// #define SCL_H         GPIOB->BSRR = 0x00000400U	//gpioPinWrite(_DEF_I2C2_SCL, GPIO_PIN_SET) 	//GPIOB->BSRR = GPIO_PIN_10	 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
// #define SCL_L         GPIOB->BSRR = 0x04000000U	//gpioPinWrite(_DEF_I2C2_SCL, GPIO_PIN_RESET)	//GPIOB->BRR  = GPIO_PIN_10  /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

// #define SDA_H         GPIOB->BSRR = 0x00000800U	//gpioPinWrite(_DEF_I2C2_SDA, GPIO_PIN_SET)		//GPIOB->BSRR = GPIO_PIN_11	 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
// #define SDA_L         GPIOB->BSRR = 0x08000000U	//gpioPinWrite(_DEF_I2C2_SDA, GPIO_PIN_RESET)	//GPIOB->BRR  = GPIO_PIN_11	 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

// #define SCL_read      digitalReadFast(BDPIN_GPIO_1)	//(GPIOB->IDR) & GPIO_PIN_10	//GPIOB->IDR  & GPIO_PIN_10	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
// #define SDA_read      digitalReadFast(BDPIN_GPIO_2)	//(GPIOB->IDR) & GPIO_PIN_11	//GPIOB->IDR  & GPIO_PIN_11	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

#define ADDR_OV7725   0x42

/* Private function prototypes -----------------------------------------------*/
// void I2C_Configuration(void);
// int I2C_WriteByte( uint16_t WriteAddress , uint8_t SendByte , uint8_t DeviceAddress);
// int I2C_ReadByte(uint8_t* pBuffer,   uint16_t length,   uint16_t ReadAddress,  uint8_t DeviceAddress);
// void SDA_input(void);
// void SDA_output(void);

int cambus_init(void);
int cambus_scan(void);
int cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data);
int cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data);

// void I2C_delay(void);

#ifdef __cplusplus
 }
#endif

#endif /* __SCCB_H */

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
