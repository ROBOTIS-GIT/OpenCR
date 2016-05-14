/*
 *  led.c
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBPH
 */

#include "led.h"





void led_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOB_CLK_ENABLE();


  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);


  // Configure GPIO pin : PB7
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void led_on( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
      break;
  }
}

void led_off( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      break;
  }
}

void led_toggle( uint8_t ch )
{
  switch( ch )
  {
    case 0:
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
      break;
  }
}

