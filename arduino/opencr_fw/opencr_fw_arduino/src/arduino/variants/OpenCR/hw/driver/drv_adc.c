/*
 *  drv_adc.c
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "drv_adc.h"
#include "variant.h"


ADC_HandleTypeDef hADC1;
ADC_HandleTypeDef hADC3;


int drv_adc_init()
{
  hADC3.Instance                   = ADC3;
  hADC3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hADC3.Init.Resolution            = ADC_RESOLUTION_12B;
  hADC3.Init.ScanConvMode          = DISABLE;
  hADC3.Init.ContinuousConvMode    = DISABLE;
  hADC3.Init.DiscontinuousConvMode = DISABLE;
  hADC3.Init.NbrOfDiscConversion   = 0;
  hADC3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hADC3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hADC3.Init.NbrOfConversion       = 1;
  hADC3.Init.DMAContinuousRequests = DISABLE;
  hADC3.Init.EOCSelection          = DISABLE;

  if (HAL_ADC_Init(&hADC3) != HAL_OK)
  {
    return -1;
  }


  hADC1.Instance                   = ADC1;
  hADC1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hADC1.Init.Resolution            = ADC_RESOLUTION_12B;
  hADC1.Init.ScanConvMode          = DISABLE;
  hADC1.Init.ContinuousConvMode    = DISABLE;
  hADC1.Init.DiscontinuousConvMode = DISABLE;
  hADC1.Init.NbrOfDiscConversion   = 0;
  hADC1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hADC1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hADC1.Init.NbrOfConversion       = 1;
  hADC1.Init.DMAContinuousRequests = DISABLE;
  hADC1.Init.EOCSelection          = DISABLE;

  if (HAL_ADC_Init(&hADC1) != HAL_OK)
  {
    return -1;
  }

  return 0;
}


void drv_adc_pin_init( uint32_t ulPin )
{
  GPIO_InitTypeDef GPIO_InitStruct;


  if( g_Pin2PortMapArray[ulPin].GPIOx_Port == NULL ) return;


  HAL_GPIO_DeInit(g_Pin2PortMapArray[ulPin].GPIOx_Port, g_Pin2PortMapArray[ulPin].Pin_abstraction);

  GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct);
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  uint8_t i;
  GPIO_InitTypeDef GPIO_InitStruct;


  i = 0;
  while(1)
  {
    if( g_Pin2PortMapArray[i].GPIOx_Port == NULL ) break;

    if( hadc->Instance == ADC3 )
    {
      __HAL_RCC_ADC3_CLK_ENABLE();
    }
    if( hadc->Instance == ADC1 )
    {
      __HAL_RCC_ADC1_CLK_ENABLE();
    }

    HAL_GPIO_DeInit(g_Pin2PortMapArray[i].GPIOx_Port, g_Pin2PortMapArray[i].Pin_abstraction);

    GPIO_InitStruct.Pin = g_Pin2PortMapArray[i].Pin_abstraction;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(g_Pin2PortMapArray[i].GPIOx_Port, &GPIO_InitStruct);

    i++;
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  uint8_t i;


  i = 0;
  while(1)
  {
    if( g_Pin2PortMapArray[i].GPIOx_Port == NULL ) break;

    if( hadc->Instance == ADC3 )
    {
      __HAL_RCC_ADC3_CLK_DISABLE();
    }
    if( hadc->Instance == ADC1 )
    {
      __HAL_RCC_ADC1_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(g_Pin2PortMapArray[i].GPIOx_Port, g_Pin2PortMapArray[i].Pin_abstraction);

    i++;
  }
}

