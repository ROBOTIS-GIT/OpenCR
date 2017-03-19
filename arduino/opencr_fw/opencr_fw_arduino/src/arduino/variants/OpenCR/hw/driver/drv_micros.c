/*
 *  drv.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_micros.h"
#include "variant.h"






TIM_HandleTypeDef    TimHandle;






void drv_micros_init()
{
  uint32_t uwPrescalerValue = 0;


  __HAL_RCC_TIM5_CLK_ENABLE();


  // Compute the prescaler value to have TIMx counter clock equal to 1Mh
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 1000000) - 1;


  TimHandle.Instance = TIM5;
  TimHandle.Init.Period            = 0xFFFFFFFF;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;


  HAL_TIM_Base_Init(&TimHandle);
  HAL_TIM_Base_Start(&TimHandle);
}


uint32_t drv_micros()
{
  return TimHandle.Instance->CNT;
}
