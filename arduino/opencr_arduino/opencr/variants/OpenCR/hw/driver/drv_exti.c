/*
 *  drv_exti.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_exti.h"
#include "variant.h"



typedef struct
{
  uint8_t  exti_enable;
  uint16_t pin_channel;
  uint16_t pin_number;
  void (*exti_callback)(void);
} exti_data_t;


volatile exti_data_t exti_data[EXTI_COUNT];


int drv_exti_init()
{
  uint32_t i;

  for( i=0; i<EXTI_COUNT; i++ )
  {
    exti_data[i].exti_enable   = 0;
    exti_data[i].exti_callback = NULL;
    exti_data[i].pin_channel   = 0;
  }

  i = 0;
  while(1)
  {
    if( g_Pin2PortMapArray[i].GPIOx_Port == NULL ) break;

    if(g_Pin2PortMapArray[i].extiChannel != NO_EXTI )
    {
      exti_data[g_Pin2PortMapArray[i].extiChannel].pin_number = i;
    }

    i++;
  }
  return 0;
}


void drv_exti_attach( uint32_t int_num, void (*callback)(void), uint32_t mode )
{
  uint32_t ulPin;
  GPIO_InitTypeDef   GPIO_InitStructure;


  if( int_num >= EXTI_COUNT ) return;

  ulPin = exti_data[int_num].pin_number;

  if( g_Pin2PortMapArray[ulPin].extiChannel == NO_EXTI ) return;

  exti_data[ g_Pin2PortMapArray[ulPin].extiChannel ].exti_enable   = 1;
  exti_data[ g_Pin2PortMapArray[ulPin].extiChannel ].pin_channel   = g_Pin2PortMapArray[ulPin].Pin_abstraction;
  exti_data[ g_Pin2PortMapArray[ulPin].extiChannel ].exti_callback = callback;


  switch( mode )
  {
    case CHANGE:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;

    case FALLING:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
      break;

    case RISING:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
      break;

    default:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
      return;
  }


  //GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin  = g_Pin2PortMapArray[ulPin].Pin_abstraction;
  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStructure);



  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_0 )
  {
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_1 )
  {
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_2 )
  {
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_3 )
  {
    HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_4 )
  {
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }

  if(    g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_5
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_6
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_7
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_8
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_9 )
  {
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }

  if(    g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_10
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_11
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_12
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_13
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_14
      || g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_15 )
  {
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
}


void drv_exti_detach( uint32_t int_num )
{
  uint32_t i;
  uint32_t ulPin;


  if( int_num >= EXTI_COUNT ) return;

  ulPin = exti_data[int_num].pin_number;


  if( g_Pin2PortMapArray[ulPin].extiChannel == NO_EXTI ) return;

  exti_data[ g_Pin2PortMapArray[ulPin].extiChannel ].exti_enable   = 0;
  exti_data[ g_Pin2PortMapArray[ulPin].extiChannel ].exti_callback = NULL;


  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_0 )
  {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_1 )
  {
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_2 )
  {
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_3 )
  {
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  }

  if( g_Pin2PortMapArray[ulPin].Pin_abstraction == GPIO_PIN_4 )
  {
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  }

  for( i=0; i<EXTI_COUNT; i++ )
  {
    if( exti_data[i].pin_channel == GPIO_PIN_5 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_6 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_7 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_8 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_9 && exti_data[i].exti_enable == 1 ) break;
  }
  if( i == EXTI_COUNT )
  {
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  }


  for( i=0; i<EXTI_COUNT; i++ )
  {
    if( exti_data[i].pin_channel == GPIO_PIN_10 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_11 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_12 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_13 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_14 && exti_data[i].exti_enable == 1 ) break;
    if( exti_data[i].pin_channel == GPIO_PIN_15 && exti_data[i].exti_enable == 1 ) break;
  }
  if( i == EXTI_COUNT )
  {
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t i;

  for( i=0; i<EXTI_COUNT; i++ )
  {
    if(GPIO_Pin == exti_data[i].pin_channel)
    {
      if(exti_data[i].exti_callback != NULL )
      {
        (*exti_data[i].exti_callback)();
      }
    }
  }
}


void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}


void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}


void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}


void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}


void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}
