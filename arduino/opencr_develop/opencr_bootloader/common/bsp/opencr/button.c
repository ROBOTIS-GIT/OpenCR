/*
 *  button.c
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "button.h"




void button_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();



  // Configure GPIO pin : PC12
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  // Configure GPIO pin : PG3
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}


uint8_t button_read( uint8_t pin_num )
{
  switch( pin_num )
  {
    case 0:
      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET ) return TRUE;
      else                                                           return FALSE;
      break;

    case 1:
      if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3) == GPIO_PIN_SET )  return TRUE;
      else                                                           return FALSE;
      break;
  }

  return FALSE;
}

