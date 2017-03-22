/*
 *  wdg.c
 *
 *  Created on: 2016. 7. 7.
 *      Author: Baram
 */

#include "wdg.h"



static IWDG_HandleTypeDef IwdgHandle;



void wdgInit()
{
}

bool wdgSetup(uint32_t reload_time)
{

  IwdgHandle.Instance 	    = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32; // 32Khz/32 = 1Khz(1ms)
  IwdgHandle.Init.Reload    = reload_time;
  IwdgHandle.Init.Window    = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    return false;
  }

  return true;
}

bool wdgStart(void)
{
  return true;
}

bool wdgGetReset(void)
{

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    // IWDGRST flag set
    // Clear reset flags
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return true;
  }
  else
  {
    // IWDGRST flag is not set
    return false;
  }
}
