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

 #include "usbd_core.h"
 #include "usbd_desc.h"
 #include "usbd_cdc.h"
 #include "usbd_cdc_interface.h"




USBD_HandleTypeDef USBD_Device;

extern void stop();


void bsp_mpu_config(void);



void bsp_init()
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



  //HAL_Delay(100);

  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);


  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
}


void bsp_deinit()
{

  USBD_DeInit(&USBD_Device);
  HAL_DeInit();
  __disable_irq();

  SCB_InvalidateDCache();
  SCB_InvalidateICache();
  SCB_DisableDCache();
  SCB_DisableICache();

}


void bsp_mpu_config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
