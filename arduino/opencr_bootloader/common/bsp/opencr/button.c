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
  __HAL_RCC_GPIOI_CLK_ENABLE();



  // Configure GPIO pin : PI11
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
}


uint8_t button_read( uint8_t pin_num )
{
  if(HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) == GPIO_PIN_SET ) return TRUE;
  else                                                        return FALSE;
}

