/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               OV7725.h
** Descriptions:            None
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2011-2-13
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             Will Son
** Modified date:           2018-5-2
** Version:                 v1.0
** Descriptions:            OV7725 for OpenCR1.0
**
*********************************************************************************************************/

#ifndef __OV7725_H
#define __OV7725_H 

/* Includes ------------------------------------------------------------------*/	   

#include "../tftLcd.h"

#define ov7725_xclk		20000000  //20Mhz
#define XCLK_PIN  BDPIN_GPIO_8  //GPIO #10(GPIOE PIN11)
#define CAM_VSYNC BDPIN_GPIO_3
#define CAM_WRST  BDPIN_GPIO_4
#define CAM_WEN   BDPIN_GPIO_5
#define CAM_RRST  BDPIN_GPIO_7
#define CAM_REN   BDPIN_GPIO_6
#define CAM_RCLK  BDPIN_GPIO_9
#define FIFO0     BDPIN_GPIO_10
#define FIFO1     BDPIN_GPIO_11
#define FIFO2     BDPIN_GPIO_12
#define FIFO3     BDPIN_GPIO_13
#define FIFO4     BDPIN_GPIO_14
#define FIFO5     BDPIN_GPIO_15
#define FIFO6     BDPIN_GPIO_16
#define FIFO7     BDPIN_GPIO_17

/*------------------------------------------------------
  ģ���������� | ����            |     STM32���������� |
  ------------------------------------------------------
  SCCB_SCL     : SCCB SCL             : PB10    I2C2_SCL
  SCCB_SDA     : SCCB SDA             : PB11    I2C2_SDA
  CAM_VSYNC    : VSYNC                : PC13    GPIO #5
  CAM_WRST     : FIFO Write Reset     : PD2     GPIO #6
  CAM_WEN      : FIFO Write Enable    : PE3     GPIO #7
  XCLK         : CMOS Clock           : PE11    GPIO #10
  CAM_RRST     : FIFO Read Reset      : PE10    GPIO #9
  CAM_REN      : FIFO Write Enable    : PG2     GPIO #8
  CAM_RCLK     : FIFO Read Clock      : PE12    GPIO #11
  FIFO D0~D7   : FIFO Data Out        : PE13/14/15,PF0/1/2,PD8,PF4   GPIO #12,13,14,15,16,17,18,19
  -----------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define FIFO_CS_PIN     GPIO_PIN_6
#define FIFO_WRST_PIN   GPIO_PIN_7
#define FIFO_RRST_PIN   GPIO_PIN_0
#define FIFO_RCLK_PIN   GPIO_PIN_1
#define FIFO_WE_PIN     GPIO_PIN_3

#define FIFO_CS_H()     GPIOG->BSRR |= 0x00000004U
#define FIFO_CS_L()     GPIOG->BSRR |= 0x00040000U

#define FIFO_WRST_H()   GPIOD->BSRR |= 0x00000004U
#define FIFO_WRST_L()   GPIOD->BSRR |= 0x00040000U

#define FIFO_RRST_H()   GPIOE->BSRR |= 0x00000400U
#define FIFO_RRST_L()   GPIOE->BSRR |= 0x04000000U

#define FIFO_RCLK_H()   GPIOE->BSRR |= 0x00001000U
#define FIFO_RCLK_L()   GPIOE->BSRR |= 0x10000000U

#define FIFO_WE_H()     GPIOE->BSRR |= 0x00000008U
#define FIFO_WE_L()     GPIOE->BSRR |= 0x00080000U

#define FIFO_OE_H()		GPIOG->BSRR |= 0x00000004U
#define FIFO_OE_L()		GPIOG->BSRR |= 0x00040000U

#define REG_NUM                            79
#define PORT_VSYNC_CMOS                    GPIOA
#define RCC_APB2Periph_PORT_VSYNC_CMOS     RCC_APB2Periph_GPIOA
#define PIN_VSYNC_CMOS                     GPIO_Pin_0
#define EXTI_LINE_VSYNC_CMOS               EXTI_Line0
#define PORT_SOURCE_VSYNC_CMOS             GPIO_PortSourceGPIOA
#define PIN_SOURCE_VSYNC_CMOS              GPIO_PinSource0

const uint8_t OV7725_Reg[REG_NUM][2] =
{
  /* Setting OV7725QVGA RGB565 */
	{0x2a,0x00}, //QVGA : {0x2a,0x00} , QQVGA : {0x2a,0x00}
	{0x11,0x00}, // 00/01/03/07 for 60/30/15/7.5fps  - set to 15fps for QVGA
	{0x0D,0xF1},
	{0x12,0x46}, /* QVGA RGB565 {0x12,0x46}, QQVGA RGB565 {0x12,0x46}*/
	{0x17,0x3F},
	{0x18,0x50},
	{0x19,0x03},
	{0x1A,0x78},
	{0x32,0x80},

	{0xAC,0xff}, //auto scaling
	{0x42,0x7f},
	{0x4d,0x00}, /* 0x09 */
	{0x63,0xf0},
	{0x64,0xff},
	{0x65,0x00}, //0x20
	{0x66,0x10}, //0x00
	{0x67,0x00},
	{0x69,0x5C},
	{0x13,0x8F},  //{0x13,0xff},
	{0x0d,0x41}, /* PLL, 0xc1*/
	{0x0f,0x43}, //0xc5
	{0x14,0x4A}, //{0x14,0x11},
	{0x22,0xFF}, // ff/7f/3f/1f for 60/30/15/7.5fps
	{0x23,0x01}, // 01/03/07/0f for 60/30/15/7.5fps
	{0x24,0x34},
	{0x25,0x3c},
	{0x26,0xa1},
	{0x2b,0x00},
	{0x6b,0xaa},

	{0x90,0x0a},
	{0x91,0x01}, //{0x91,0x01}
	{0x92,0x01}, //{0x92,0x01}
	{0x93,0x01},

	{0x94,0x5f}, //{0x94,0x5f}
	{0x95,0x53}, //{0x95,0x53}
	{0x96,0x11}, //{0x96,0x11}
	{0x97,0x1a}, //{0x97,0x1a}
	{0x98,0x3d}, //{0x98,0x3d}
	{0x99,0x5a}, //{0x99,0x5a}
	{0x9a,0x1e}, //{0x9a,0x1e}

	{0x9b,0x00}, /* set luma */
	{0x9c,0x25}, /* set contrast */
	{0xa7,0x40}, /* set saturation {0xa7,0x65}*/
	{0xa8,0x40}, /* set saturation {0xa8,0x65}*/
	{0xa9,0x80}, /* set hue */
	{0xaa,0x80}, /* set hue */

	{0x9e,0x81},
	{0xa6,0x00},
	{0x7e,0x0c},
	{0x7f,0x16},
	{0x80,0x2a},
	{0x81,0x4e},
	{0x82,0x61},
	{0x83,0x6f},
	{0x84,0x7b},
	{0x85,0x86},
	{0x86,0x8e},
	{0x87,0x97},
	{0x88,0xa4},
	{0x89,0xaf},
	{0x8a,0xc5},
	{0x8b,0xd7},
	{0x8c,0xe8},
	{0x8d,0x20},
	{0x33,0x00},
	{0x22,0x99},
	{0x23,0x03},
	{0x4a,0x00},
	{0x49,0x50}, //{0x49,0x13}
	{0x47,0x08}, //{0x47,0x08}
	{0x4b,0x14}, //{0x4b,0x14}
	{0x4c,0x17}, //{0x4c,0x17}
	{0x46,0x00}, //{0x46,0x05}
	{0x0e,0x01}, //{0x0e,0xf5}1111,0101
	{0x0c,0xD0}, //bit6 : Horizontal Mirror, bit0 : test pattern enable
	
	{0x29,0x50},	//_USE_QVGA
	{0x2c,0x78},	//_USE_QVGA

	{0x29,0x28},	//_USE_QQVGA
	{0x2c,0x3C},	//_USE_QQVGA
};
/* Private variables ---------------------------------------------------------*/	

/* Private function prototypes -----------------------------------------------*/
int  OV7725_Init(void);
uint8_t getVsync(void);
void setVsync(uint8_t value);
void readIMG(uint8_t size);
void setRawImgSize(uint8_t value);

#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
