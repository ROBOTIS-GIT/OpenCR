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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();



  // Configure GPIO pin : PG11
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);


  // Configure GPIO pin : PE6
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}


uint8_t button_read( uint8_t pin_num )
{
  switch( pin_num )
  {
    case 0:
      if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11) == GPIO_PIN_SET ) return TRUE;
      else                                                           return FALSE;
      break;

    case 1:
      if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET )  return TRUE;
      else                                                           return FALSE;
      break;
  }

  return FALSE;
}

