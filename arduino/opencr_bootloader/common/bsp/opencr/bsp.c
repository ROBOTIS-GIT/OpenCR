#include "bsp.h"


USBD_HandleTypeDef USBD_Device;



void bsp_init()
{
  // STM32Cube HAL Init
  HAL_Init();

  // Clock Setup
  // SYSCLK(Hz)    = 216000000
  // HCLK(Hz)      = 216000000
  // HSE(Hz)       = 25000000
  SystemClock_Config();


  led_init();
  button_init();


  /*##-6- Enable TIM peripherals Clock #######################################*/
  //TIMx_CLK_ENABLE();
  //HAL_NVIC_SetPriority(TIMx_IRQn, 6, 0);
  //HAL_NVIC_EnableIRQ(TIMx_IRQn);


  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
}
