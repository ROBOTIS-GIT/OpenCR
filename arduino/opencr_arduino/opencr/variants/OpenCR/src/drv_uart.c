/*
 *  drv_uart.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_uart.h"
#include "variant.h"


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


int drv_uart_init()
{



  return 0;
}


void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}


void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if( UartHandle->Instance == USART6 ) Tx1_Handler();
  if( UartHandle->Instance == USART2 ) Tx2_Handler();
  if( UartHandle->Instance == USART3 ) Tx3_Handler();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  __HAL_UART_FLUSH_DRREGISTER(UartHandle);

  if( UartHandle->Instance == USART6 ) Rx1_Handler();
  if( UartHandle->Instance == USART2 ) Rx2_Handler();
  if( UartHandle->Instance == USART3 ) Rx3_Handler();
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  if( UartHandle->Instance == USART6 ) Err1_Handler();
  if( UartHandle->Instance == USART2 ) Err2_Handler();
  if( UartHandle->Instance == USART3 ) Err3_Handler();
}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;


  if(huart->Instance==USART2)
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    RCC_PeriphCLKInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART2_IRQn);
  }
  else if(huart->Instance==USART6)
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    RCC_PeriphCLKInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    GPIO_InitStruct.Pin       = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART6_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    RCC_PeriphCLKInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART3_IRQn);
  }
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART2)
  {
    __USART2_FORCE_RESET();
    __USART2_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
  else if(huart->Instance==USART6)
  {
    __USART6_FORCE_RESET();
    __USART6_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_7);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    __USART3_FORCE_RESET();
    __USART3_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}
