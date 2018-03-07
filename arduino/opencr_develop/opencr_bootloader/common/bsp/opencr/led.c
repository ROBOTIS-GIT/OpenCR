/*
 *  led.c
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "led.h"

#define LED_MAX   1


GPIO_TypeDef *pLedGpio[LED_MAX] = { GPIOG };
uint16_t      LedGpioPin[LED_MAX] = { GPIO_PIN_9 };

//GPIO_TypeDef *pLedGpio[1] = { GPIOB };
//uint16_t      LedGpioPin[1] = { GPIO_PIN_7 };



void led_init()
{
  uint8_t i;
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOG_CLK_ENABLE();


  for( i=0; i<LED_MAX; i++ )
  {
    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(pLedGpio[i], LedGpioPin[i], GPIO_PIN_RESET);

    // Configure GPIO pin : PB7
    GPIO_InitStruct.Pin = LedGpioPin[i];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(pLedGpio[i], &GPIO_InitStruct);
  }
}

void led_on( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_WritePin(pLedGpio[ch], LedGpioPin[ch], GPIO_PIN_SET);
      break;
  }
}

void led_off( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_WritePin(pLedGpio[ch], LedGpioPin[ch], GPIO_PIN_RESET);
      break;
  }
}

void led_toggle( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_TogglePin(pLedGpio[ch], LedGpioPin[ch]);
      break;
  }
}

