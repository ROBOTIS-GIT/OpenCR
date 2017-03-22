/*
 *  led.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */

#include "led.h"



typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin_number;
  GPIO_PinState on_state;
  GPIO_PinState off_state;
} led_port_t;


static led_port_t led_port_tbl[LED_CH_MAX] =
{
    {GPIOG, GPIO_PIN_12, GPIO_PIN_RESET, GPIO_PIN_SET},
    {GPIOE, GPIO_PIN_5 , GPIO_PIN_RESET, GPIO_PIN_SET},
    {GPIOE, GPIO_PIN_4 , GPIO_PIN_RESET, GPIO_PIN_SET},
    {GPIOG, GPIO_PIN_10, GPIO_PIN_RESET, GPIO_PIN_SET}
};







void ledInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint32_t i;


  for (i=0; i<LED_CH_MAX; i++)
  {
    GPIO_InitStruct.Pin  = led_port_tbl[i].pin_number;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(led_port_tbl[i].port, &GPIO_InitStruct);

    ledOff(i);
  }
}

void ledOn(uint8_t ch)
{
  if (ch >= LED_CH_MAX) return;

  HAL_GPIO_WritePin(led_port_tbl[ch].port, led_port_tbl[ch].pin_number, led_port_tbl[ch].on_state);
}

void ledOff(uint8_t ch)
{
  if (ch >= LED_CH_MAX) return;

  HAL_GPIO_WritePin(led_port_tbl[ch].port, led_port_tbl[ch].pin_number, led_port_tbl[ch].off_state);
}

void ledToggle(uint8_t ch)
{
  if (ch >= LED_CH_MAX) return;

  HAL_GPIO_TogglePin(led_port_tbl[ch].port, led_port_tbl[ch].pin_number);
}
