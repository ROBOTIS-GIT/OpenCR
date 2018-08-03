/*
 *  drv_spi.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_i2c.h"
#include "variant.h"


I2C_HandleTypeDef drv_i2c_handles[DRV_I2C_CNT];     // If we are doing hardware I2C
I2C_TypeDef *i2c_instance[DRV_I2C_CNT] = {I2C1, I2C2};

int drv_i2c_init()
{
  for (int i=0; i < DRV_I2C_CNT; i++) 
  {
    drv_i2c_handles[i].Instance = i2c_instance[i];
  }
  return 0;
}


void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;


  if( hi2c->Instance == I2C1 )
  {
    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Enable I2Cx clock */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /*##-3- Configure peripheral GPIO ##########################################*/
    /* I2C SCL GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C SDA GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ  (I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ  (I2C1_ER_IRQn);

  }
  if( hi2c->Instance == I2C2 )
  {
    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    RCC_PeriphCLKInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Enable I2Cx clock */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /*##-3- Configure peripheral GPIO ##########################################*/
    /* I2C SCL GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C SDA GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ  (I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ  (I2C2_ER_IRQn);
  }
}

/**
  * @brief I2C MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if( hi2c->Instance == I2C1 )
  {
    /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure I2C SCL as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    /* Configure I2C SDA as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ  (I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ  (I2C1_ER_IRQn);
  }
  if( hi2c->Instance == I2C2 )
  {
    /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure I2C SCL as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
    /* Configure I2C SDA as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ  (I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ  (I2C2_ER_IRQn);
  }
}

void I2C1_EV_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler( &drv_i2c_handles[0]);
}

void I2C2_EV_IRQHandler(void)  {
  HAL_I2C_EV_IRQHandler( &drv_i2c_handles[1]);
}

void I2C1_ER_IRQHandler(void) {
  HAL_I2C_ER_IRQHandler( &drv_i2c_handles[0]);
}

void I2C2_ER_IRQHandler(void)  {
  HAL_I2C_ER_IRQHandler( &drv_i2c_handles[1]);
}
