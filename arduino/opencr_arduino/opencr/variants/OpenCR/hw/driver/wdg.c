/*
 *  wdg.c
 *
 *  Created on: 2016. 7. 7.
 *      Author: Baram, PBHP
 */

#include "wdg.h"



static IWDG_HandleTypeDef IwdgHandle;
static BOOL is_setup = FALSE;


void wdg_init()
{
}

BOOL wdg_setup(uint32_t reload_time)
{

  IwdgHandle.Instance 	    = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32; // 32Khz/32 = 1Khz(1ms)
  IwdgHandle.Init.Reload    = reload_time;
  IwdgHandle.Init.Window    = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    return FALSE;
  }

  is_setup = TRUE;

  return TRUE;
}

BOOL wdg_start(void)
{
  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    return FALSE;
  }

  return TRUE;
}

BOOL wdg_refresh(void)
{
  if(is_setup == FALSE)
  {
    return FALSE;
  }

  if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
  {
    return FALSE;
  }

  return TRUE;
}

BOOL wdg_get_reset(void)
{

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    // IWDGRST flag set
    // Clear reset flags
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return TRUE;
  }
  else
  {
    // IWDGRST flag is not set
    return FALSE;
  }
}
