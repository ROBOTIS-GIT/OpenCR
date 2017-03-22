/*
 *  bsp.c
 *
 *  boart support package
 *
 *  Created on: 2017. 3. 16.
 *      Author: Baram
 */
 #include "bsp.h"

 #include "hw.h"








void bspInit()
{
  // STM32Cube HAL Init
  HAL_Init();

  // Clock Setup
  // SYSCLK(Hz)    = 216000000
  // HCLK(Hz)      = 216000000
  // HSE(Hz)       = 25000000
  SystemClock_Config();

  SCB_EnableDCache();
  SCB_EnableICache();


  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

 }


void bspDeinit()
{
  HAL_DeInit();
}
