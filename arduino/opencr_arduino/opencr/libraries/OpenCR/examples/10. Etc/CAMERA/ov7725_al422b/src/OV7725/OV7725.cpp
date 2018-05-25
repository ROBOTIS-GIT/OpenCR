/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               OV7725.c
** Descriptions:            OV7725 application function
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

/* Includes ------------------------------------------------------------------*/
#include "OV7725.h"
#include "../sccb/sccb.h"
#include "../../settings.h"

volatile uint8_t Vsync = 0;
uint8_t rawImgSize = QVGA;
uint16_t firstHData = 0;
uint16_t secondHData = 0;
uint16_t thirdHData = 0;
uint16_t firstLData = 0;
uint16_t secondLData = 0;
uint16_t thirdLData = 0;
uint16_t cmosData = 0;
uint8_t  camAddress = 0;

static uint8_t toggle = 0;

GPIO_TypeDef *first;
GPIO_TypeDef *second;
GPIO_TypeDef *third;

static void vsync_interrupt(void)
{
  if(Vsync == 0)
  {
    //FIFO Write Reset
    FIFO_WRST_L();
    FIFO_WE_H();
    Vsync = 1;
    FIFO_WE_H();
    FIFO_WRST_H();
  }
  else if(Vsync == 1)
  {
    FIFO_WE_L();
    Vsync = 2;
  }
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
}

/*******************************************************************************
* Function Name  : OV7725_XCLK_Init
* Description    : Set general GPIO pin as PWM output and use for clock as MCOx is not available
* Input          : GPIO pin, Resolution, Duty
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void OV7725_XCLK_Init(void)
{
  if(drv_pwm_get_init(XCLK_PIN) == false)
  {
    drv_pwm_set_freq(XCLK_PIN, ov7725_xclk);
    drv_pwm_setup(XCLK_PIN);
  }
  drv_pwm_set_duty(XCLK_PIN, 6, 32);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : OV7725 VSYNC GPIO Configuration
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/            
static void GPIO_Configuration(void)
{
  pinMode(CAM_VSYNC, INPUT_PULLDOWN);
  attachInterrupt(9, vsync_interrupt, RISING);  //attach rising edge detection interrupt on VSYNC
}

/*******************************************************************************
* Function Name  : FIFO_GPIO_Configuration
* Description    : AL422B FIFO control GPIO Configuration 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/  
static void FIFO_GPIO_Configuration(void)
{
  /* FIFO_RCLK : PE1 */
  pinMode(CAM_RCLK, OUTPUT);

  /* FIFO_RRST : PE0 */
  pinMode(CAM_RRST, OUTPUT);

  /* FIFO_CS or REN : PD6 */
  pinMode(CAM_REN, OUTPUT);

  /* FIFO_WEN : PD3 */
  pinMode(CAM_WEN, OUTPUT);

  /* FIFO_WRST : PB7 */
  pinMode(CAM_WRST, OUTPUT);

  /* FIFO D[0-7] */
  pinMode(FIFO0, INPUT_PULLUP);
  pinMode(FIFO1, INPUT_PULLUP);
  pinMode(FIFO2, INPUT_PULLUP);
  pinMode(FIFO3, INPUT_PULLUP);
  pinMode(FIFO4, INPUT_PULLUP);
  pinMode(FIFO5, INPUT_PULLUP);
  pinMode(FIFO6, INPUT_PULLUP);
  pinMode(FIFO7, INPUT_PULLUP);

  first = g_Pin2PortMapArray[FIFO0].GPIOx_Port;
  second = g_Pin2PortMapArray[FIFO3].GPIOx_Port;
  third = g_Pin2PortMapArray[FIFO6].GPIOx_Port;
}

/*******************************************************************************
* Function Name  : OV7725_Init
* Description    : OV7725��ʼ��
* Input          : None
* Output         : None
* Return         : None
* Attention		 : ����1�ɹ�������0ʧ��
*******************************************************************************/ 
int OV7725_Init(void)
{
  int16_t i=0;

  OV7725_XCLK_Init();
  GPIO_Configuration();
  FIFO_GPIO_Configuration();

  if(cambus_init() != 0)
  {
	  return 1;	//init error
  }

  camAddress = cambus_scan();	//OV7725 Address = 0x42

  // Reset SCCB
  if(camAddress == ADDR_OV7725)
  {
    if(cambus_writeb(ADDR_OV7725, 0x12, 0x80) != 0)
    {
      return 1;
    }
  }
  delay_ms(10);

  //Setting OV7725 Register
  for( i=0 ; i < (REG_NUM - 2) ; i++ )
  {
	  if( cambus_writeb(ADDR_OV7725, OV7725_Reg[i][0], OV7725_Reg[i][1]) != 0 )
	  {
		  return 1;
	  }
	  delay_ms(10);
  }

  if(rawImgSize == QQVGA)
  {
    cambus_writeb(ADDR_OV7725, 0x29, 0x28);
    delay_ms(10);
    cambus_writeb(ADDR_OV7725, 0x2c, 0x3c);
  }

  return 0;
}

uint8_t getVsync(void)
{
  return Vsync;
}

void setVsync(uint8_t value)
{
  Vsync = value;
}

void setRawImgSize(uint8_t value)
{
  if (value == QVGA)
    rawImgSize = QVGA;
  else if (value == QQVGA)
    rawImgSize = QQVGA;
}

// Read Image from FIFO
void readIMG(uint8_t size) 
{
  uint32_t pixelCount = 0;
  uint32_t qtPixelCount = 0;
  float    fWeight = 0;

  FIFO_WE_L();

  FIFO_RRST_L();

  FIFO_RCLK_H();  //Read Reset
  FIFO_RCLK_L();
  
  FIFO_RCLK_H();  //Read Reset
  FIFO_RCLK_L();

  FIFO_RRST_H();

  if(size == QUARTERVIEW)
  {
    for(pixelCount = 0; pixelCount < (LCD_WIDTH*LCD_HEIGHT) ;pixelCount++)
    {
      if((pixelCount % 2 == 0) && ((pixelCount / 320) % 2 == 0))
      {
        FIFO_RCLK_H();

        //High Byte
        FIFO_RCLK_L();
        firstHData = first->IDR; 	  //XXX0 0000 0000 0000
        secondHData = second->IDR; 	//0000 0000 000X 0XXX
        thirdHData = third->IDR;		//0000 000X 0000 0000
        FIFO_RCLK_H();

        //Low Byte
        FIFO_RCLK_L();
        firstLData = first->IDR; 	  //XXX0 0000 0000 0000
        secondLData = second->IDR; 	//0000 0000 000X 0XXX
        thirdLData = third->IDR;	  //0000 000X 0000 0000
        FIFO_RCLK_H();

        //1st and 2nd byte is inverted. therefore, GGGBBBBB RRRRRGGG
        cmosData = (((firstLData >> 5) & 0x0700) | ((secondLData << 11) & 0xB800) | ((thirdLData << 6) & 0x4000) | ((firstHData >> 13) & 0x0007) | ((secondHData << 3) & 0x00B8) | ((thirdHData >> 2) & 0x0040));
        image_buf[qtPixelCount++] = cmosData;
      }
      //read and waste unnecessary data
      else
      {
        FIFO_RCLK_H();

        //High Byte
        FIFO_RCLK_L();
        firstHData = first->IDR; 	  //XXX0 0000 0000 0000
        secondHData = second->IDR; 	//0000 0000 000X 0XXX
        thirdHData = third->IDR;		//0000 000X 0000 0000
        FIFO_RCLK_H();

        //Low Byte
        FIFO_RCLK_L();
        firstLData = first->IDR; 	  //XXX0 0000 0000 0000
        secondLData = second->IDR; 	//0000 0000 000X 0XXX
        thirdLData = third->IDR;	  //0000 000X 0000 0000
        FIFO_RCLK_H();
      }
    }
  }
  else if(size == QVGA)
  {
    for(pixelCount = 0; pixelCount < (LCD_WIDTH*LCD_HEIGHT) ;pixelCount++)
    {
      FIFO_RCLK_H();

      //High Byte
      FIFO_RCLK_L();
      firstHData = first->IDR; 	  //XXX0 0000 0000 0000
      secondHData = second->IDR; 	//0000 0000 000X 0XXX
      thirdHData = third->IDR;		//0000 000X 0000 0000
      FIFO_RCLK_H();

      //Low Byte
      FIFO_RCLK_L();
      firstLData = first->IDR; 	  //XXX0 0000 0000 0000
      secondLData = second->IDR; 	//0000 0000 000X 0XXX
      thirdLData = third->IDR;	  //0000 000X 0000 0000
      FIFO_RCLK_H();

      //1st and 2nd byte is inverted. therefore, GGGBBBBB RRRRRGGG
      cmosData = (((firstLData >> 5) & 0x0700) | ((secondLData << 11) & 0xB800) | ((thirdLData << 6) & 0x4000) | ((firstHData >> 13) & 0x0007) | ((secondHData << 3) & 0x00B8) | ((thirdHData >> 2) & 0x0040));
      image_buf[pixelCount] = cmosData;
    }
  }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
