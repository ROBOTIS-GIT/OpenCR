/*
 *  drv_uart.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */
/*

  USART6
    - RX : DMA2, Channel 5, Stream 2
    - TX : DMA2, Channel 5, Stream 6

  USART2
    - RX : DMA1, Channel 4, Stream 5
    - TX : DMA1, Channel 4, Stream 6

  USART3
    - RX : DMA1, Channel 4, Stream 1
    - TX : DMA1, Channel 4, Stream 3

  USART8
    - RX : DMA1, Channel 5, Stream 6
    - TX : DMA1, Channel 5, Stream 0
*/
#include "drv_uart.h"
#include "variant.h"
#include "dma_stream_handlers.h"


//-- internal definition
//
#define DRV_UART_RX_BUF_LENGTH      1024


//-- internal variable
//
static uint32_t drv_uart_rx_buf_head[DRV_UART_NUM_MAX];
static uint32_t drv_uart_rx_buf_tail[DRV_UART_NUM_MAX];
static uint8_t  drv_uart_rx_buf[DRV_UART_NUM_MAX][DRV_UART_RX_BUF_LENGTH] __attribute__((section(".NoneCacheableMem")));


static BOOL is_init[DRV_UART_NUM_MAX];
static BOOL is_uart_mode[DRV_UART_NUM_MAX];
static BOOL is_uart_write_dma_mode[DRV_UART_NUM_MAX];

UART_HandleTypeDef huart[DRV_UART_NUM_MAX];
DMA_HandleTypeDef  hdma_rx[DRV_UART_NUM_MAX];
DMA_HandleTypeDef  hdma_tx[DRV_UART_NUM_MAX];
USART_TypeDef     *huart_inst[DRV_UART_NUM_MAX] = { USART6, USART2, USART3, UART8 };


//-- internal functions definition
//
void drv_uart_err_handler(uint8_t uart_num);




int drv_uart_init()
{
  uint8_t i;


  for(i=0; i<DRV_UART_NUM_MAX; i++)
  {
    is_init[i]      = FALSE;
    is_uart_mode[i] = DRV_UART_IRQ_MODE;
    is_uart_write_dma_mode[i] = false;  // assume IT mode

    drv_uart_rx_buf_head[i] = 0;
    drv_uart_rx_buf_tail[i] = 0;
  }

  return 0;
}

void drv_uart_begin(uint8_t uart_num, uint8_t uart_mode, uint32_t baudrate)
{
  if(uart_num < DRV_UART_NUM_MAX)
  {
    huart[uart_num].Instance          = huart_inst[uart_num];
    huart[uart_num].Init.BaudRate     = baudrate;
    huart[uart_num].Init.WordLength   = UART_WORDLENGTH_8B;
    huart[uart_num].Init.StopBits     = UART_STOPBITS_1;
    huart[uart_num].Init.Parity       = UART_PARITY_NONE;
    huart[uart_num].Init.Mode         = UART_MODE_TX_RX;
    huart[uart_num].Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart[uart_num].Init.OverSampling = UART_OVERSAMPLING_16;
    huart[uart_num].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    is_uart_mode[uart_num] = uart_mode;  // remember if we are DMA or not
    HAL_UART_Init(&huart[uart_num]);

    is_init[uart_num] = TRUE;

    drv_uart_start_rx(uart_num);

    if (hdma_rx[uart_num].Instance) 
    {
      // Only set if DMA instance is set, else can leave alone as tail will be set to 
      // whatever this one is... 
      drv_uart_rx_buf_head[uart_num] = DRV_UART_RX_BUF_LENGTH - hdma_rx[uart_num].Instance->NDTR;
    }
    drv_uart_rx_buf_tail[uart_num] = drv_uart_rx_buf_head[uart_num];

  }
}

uint32_t drv_uart_write(uint8_t uart_num, const uint8_t wr_data)
{
  HAL_UART_Transmit(&huart[uart_num], (uint8_t *)&wr_data, 1, 10);
  return 1;
}

uint32_t drv_uart_write_dma_it(uint8_t uart_num, const uint8_t *wr_data, uint16_t Size)
{
  // call the DMA or IT function depending on if configured for DMA or not.
  if (is_uart_write_dma_mode[uart_num]) 
  {
    return (uint32_t)HAL_UART_Transmit_DMA(&huart[uart_num], (uint8_t *)wr_data, Size);  
  }
  return (uint32_t)HAL_UART_Transmit_IT(&huart[uart_num], (uint8_t *)wr_data, Size);  
}


void drv_uart_flush(uint8_t uart_num)
{
  UNUSED(uart_num);
}

// Only called in DMA case. 
void drv_uart_rx_flush(uint8_t uart_num, uint32_t timeout_ms)
{
  uint32_t pre_time_ms = millis();
  while((drv_uart_read(uart_num) != -1) || (millis() - pre_time_ms < timeout_ms))
  {
  }
}


void drv_uart_start_rx(uint8_t uart_num)
{
#ifdef DRV_UART_RX_DMA_ONLY
  if(is_uart_mode[uart_num] == DRV_UART_IRQ_MODE)
  {
    HAL_UART_Receive_IT(&huart[uart_num], (uint8_t *)drv_uart_rx_buf[uart_num], 1);
  }
  else
  {
    HAL_UART_Receive_DMA(&huart[uart_num], (uint8_t *)drv_uart_rx_buf[uart_num], DRV_UART_RX_BUF_LENGTH );
  }
#else
  HAL_UART_Receive_DMA(&huart[uart_num], (uint8_t *)drv_uart_rx_buf[uart_num], DRV_UART_RX_BUF_LENGTH );
#endif  
}

uint32_t drv_uart_read_buf(uint8_t uart_num, uint8_t *p_buf, uint32_t length)
{
  uint32_t ret = 0;
#ifdef DRV_UART_RX_DMA_ONLY
  uint32_t i;

  if(is_uart_mode[uart_num] == DRV_UART_IRQ_MODE)
  {
    for( i=0; i<length; i++)
    {
      p_buf[i] = drv_uart_rx_buf[uart_num][i];
    }
    ret = length;
  }
  else
  {

  }
#else
  UNUSED(uart_num);
  UNUSED(p_buf);
  UNUSED(length);  
#endif
  return ret;
}

uint8_t drv_uart_get_mode(uint8_t uart_num)

{
  return is_uart_mode[uart_num];
}
// Only called in DMA mode
uint32_t  drv_uart_available(uint8_t uart_num)
{
  uint32_t length = 0;

  drv_uart_rx_buf_head[uart_num] = DRV_UART_RX_BUF_LENGTH - hdma_rx[uart_num].Instance->NDTR;

  length = (   DRV_UART_RX_BUF_LENGTH
             + drv_uart_rx_buf_head[uart_num]
             - drv_uart_rx_buf_tail[uart_num] ) % DRV_UART_RX_BUF_LENGTH;

  return length;
}

// Only called in DMA mode
int drv_uart_read(uint8_t uart_num)
{
    int ret = -1;
    int index;

    // Need to update head like available does - DMA updates it...
    drv_uart_rx_buf_head[uart_num] = DRV_UART_RX_BUF_LENGTH - hdma_rx[uart_num].Instance->NDTR;
    if (drv_uart_rx_buf_head[uart_num] != drv_uart_rx_buf_tail[uart_num])
    {
      index = drv_uart_rx_buf_tail[uart_num];
      ret = drv_uart_rx_buf[uart_num][index];
      drv_uart_rx_buf_tail[uart_num] = (drv_uart_rx_buf_tail[uart_num] + 1) % DRV_UART_RX_BUF_LENGTH;
    }
    return ret;
}

// Only called in DMA mode
int drv_uart_peek(uint8_t uart_num)
{
    int ret = -1;
    int index;

    // Need to update head like available does - DMA updates it...
    drv_uart_rx_buf_head[uart_num] = DRV_UART_RX_BUF_LENGTH - hdma_rx[uart_num].Instance->NDTR;
    if (drv_uart_rx_buf_head[uart_num] != drv_uart_rx_buf_tail[uart_num])
    {
      index = drv_uart_rx_buf_tail[uart_num];
      ret = drv_uart_rx_buf[uart_num][index];
    }
    return ret;
}

void drv_uart_err_handler(uint8_t uart_num)
{
#ifdef DRV_UART_RX_DMA_ONLY
  if(is_uart_mode[uart_num] == DRV_UART_IRQ_MODE)
  {
    drv_uart_start_rx(uart_num);
  }
  else
  {

  }
#else
  drv_uart_start_rx(uart_num);
#endif  
}




void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart[DRV_UART_NUM_1]);
}


void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart[DRV_UART_NUM_2]);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart[DRV_UART_NUM_3]);
}

void UART8_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart[DRV_UART_NUM_4]);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_1] ) Tx1_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_2] ) Tx2_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_3] ) Tx3_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_4] ) Tx4_Handler();

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  __HAL_UART_FLUSH_DRREGISTER(UartHandle);
#ifdef DRV_UART_RX_DMA_ONLY
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_1] ) Rx1_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_2] ) Rx2_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_3] ) Rx3_Handler();
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_4] ) Rx4_Handler();
#endif
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_1] ) drv_uart_err_handler(DRV_UART_NUM_1);
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_2] ) drv_uart_err_handler(DRV_UART_NUM_2);
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_3] ) drv_uart_err_handler(DRV_UART_NUM_3);
  if( UartHandle->Instance == huart_inst[DRV_UART_NUM_4] ) drv_uart_err_handler(DRV_UART_NUM_4);
}

#if 0
// UART2 DMA IRQ
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart[DRV_UART_NUM_2].hdmarx);
}

// UART3 DMA IRQ
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart[DRV_UART_NUM_3].hdmarx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart[DRV_UART_NUM_3].hdmatx);
}
#endif

// In case SPI2 wishes to use DMA and we were already
// using it for Serial3.  Downgrade Serial3 to IT mode...
void HAL_UART_LoseDMAHandler(uint8_t iStream)
{
  if (iStream == 3)
  {
    drv_uart_flush(DRV_UART_NUM_3); // note does not do anything currently...
    is_uart_write_dma_mode[DRV_UART_NUM_3] = false;
  }
  else if (iStream == 6)
  {
    drv_uart_flush(DRV_UART_NUM_2); // note does not do anything currently...
    is_uart_write_dma_mode[DRV_UART_NUM_2] = false;
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;


  if(huart->Instance==USART6) // // UART_NUM_1
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

#ifdef DRV_UART_RX_DMA_ONLY
    if(is_uart_mode[DRV_UART_NUM_1] == DRV_UART_DMA_MODE) 
#endif
    {
      // DMA Setup
      /* Configure the DMA handler for reception process */
      __HAL_RCC_DMA2_CLK_ENABLE();
      hdma_rx[DRV_UART_NUM_1].Instance                 = DMA2_Stream2;
      hdma_rx[DRV_UART_NUM_1].Init.Channel             = DMA_CHANNEL_5;
      hdma_rx[DRV_UART_NUM_1].Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma_rx[DRV_UART_NUM_1].Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_rx[DRV_UART_NUM_1].Init.MemInc              = DMA_MINC_ENABLE;
      hdma_rx[DRV_UART_NUM_1].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_rx[DRV_UART_NUM_1].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma_rx[DRV_UART_NUM_1].Init.Mode                = DMA_CIRCULAR;
      hdma_rx[DRV_UART_NUM_1].Init.Priority            = DMA_PRIORITY_HIGH;

      HAL_DMA_Init(&hdma_rx[DRV_UART_NUM_1]);

      /* Associate the initialized DMA handle to the the UART handle */
      __HAL_LINKDMA(huart, hdmarx, hdma_rx[DRV_UART_NUM_1]);

      // TELL DMA ISR handler the handle to use during ISRs...
      SetDMA2StreamHandlerHandle(2, &hdma_rx[DRV_UART_NUM_1], true, NULL);
 
      /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
      HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);


#ifndef DRV_UART_RX_DMA_ONLY
      if(is_uart_mode[DRV_UART_NUM_1] == DRV_UART_DMA_MODE) 
#endif
      {
        __HAL_RCC_DMA2_CLK_ENABLE();
        hdma_tx[DRV_UART_NUM_1].Instance                 = DMA2_Stream6;
        hdma_tx[DRV_UART_NUM_1].Init.Channel             = DMA_CHANNEL_5;
        hdma_tx[DRV_UART_NUM_1].Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx[DRV_UART_NUM_1].Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx[DRV_UART_NUM_1].Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx[DRV_UART_NUM_1].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_1].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_1].Init.Mode                = DMA_NORMAL;
        hdma_tx[DRV_UART_NUM_1].Init.Priority            = DMA_PRIORITY_MEDIUM;

        HAL_DMA_Init(&hdma_tx[DRV_UART_NUM_1]);

        /* Associate the initialized DMA handle to the the UART handle */
        __HAL_LINKDMA(huart, hdmatx, hdma_tx[DRV_UART_NUM_1]);

        // TELL DMA ISR handler the handle to use during ISRs...
        is_uart_write_dma_mode[DRV_UART_NUM_1] = SetDMA2StreamHandlerHandle(6, &hdma_tx[DRV_UART_NUM_1], false, NULL);
        if (is_uart_write_dma_mode[DRV_UART_NUM_1])
        {
          /* NVIC configuration for DMA transfer complete interrupt (USART8_RX) */
          HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
          HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
        }
        else 
        {
          vcp_printf(" Serial1 TX not DMA\n");
        }

      }
    }
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART6_IRQn);

  }
  else if(huart->Instance==USART2)  // // UART_NUM_2
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    RCC_PeriphCLKInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    // DMA Setup
    /* Configure the DMA handler for reception process */
    hdma_rx[DRV_UART_NUM_2].Instance                 = DMA1_Stream5;
    hdma_rx[DRV_UART_NUM_2].Init.Channel             = DMA_CHANNEL_4;
    hdma_rx[DRV_UART_NUM_2].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx[DRV_UART_NUM_2].Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx[DRV_UART_NUM_2].Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx[DRV_UART_NUM_2].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx[DRV_UART_NUM_2].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx[DRV_UART_NUM_2].Init.Mode                = DMA_CIRCULAR;
    hdma_rx[DRV_UART_NUM_2].Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx[DRV_UART_NUM_2]);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_rx[DRV_UART_NUM_2]);

    // TELL DMA ISR handler the handle to use during ISRs...
    SetDMA1StreamHandlerHandle(5, &hdma_rx[DRV_UART_NUM_2], false, NULL);

    /* NVIC configuration for DMA transfer complete interrupt (USART6_RX) */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    if(is_uart_mode[DRV_UART_NUM_2] == DRV_UART_DMA_MODE)
    {
      // TELL DMA ISR handler the handle to use during ISRs...
      // If this fails, than someone else is already holding onto it... So let them keep it.
      is_uart_write_dma_mode[DRV_UART_NUM_2] = SetDMA1StreamHandlerHandle(6, &hdma_tx[DRV_UART_NUM_2], false, &HAL_UART_LoseDMAHandler);

      if (is_uart_write_dma_mode[DRV_UART_NUM_2])
      {

        hdma_tx[DRV_UART_NUM_2].Instance                 = DMA1_Stream6;
        hdma_tx[DRV_UART_NUM_2].Init.Channel             = DMA_CHANNEL_4;
        hdma_tx[DRV_UART_NUM_2].Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx[DRV_UART_NUM_2].Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx[DRV_UART_NUM_2].Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx[DRV_UART_NUM_2].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_2].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_2].Init.Mode                = DMA_NORMAL;
        hdma_tx[DRV_UART_NUM_2].Init.Priority            = DMA_PRIORITY_MEDIUM;

        HAL_DMA_Init(&hdma_tx[DRV_UART_NUM_2]);

        /* Associate the initialized DMA handle to the the UART handle */
        __HAL_LINKDMA(huart, hdmatx, hdma_tx[DRV_UART_NUM_2]);

        /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
      }
      else 
      {
        //vcp_printf(" Serial2 TX not DMA\n");
      }
    }

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART2_IRQn);
  }
  else if(huart->Instance==USART3)  // // UART_NUM_3
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    RCC_PeriphCLKInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
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


    // DMA Setup
    /* Configure the DMA handler for reception process */
    hdma_rx[DRV_UART_NUM_3].Instance                 = DMA1_Stream1;
    hdma_rx[DRV_UART_NUM_3].Init.Channel             = DMA_CHANNEL_4;
    hdma_rx[DRV_UART_NUM_3].Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx[DRV_UART_NUM_3].Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx[DRV_UART_NUM_3].Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx[DRV_UART_NUM_3].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx[DRV_UART_NUM_3].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx[DRV_UART_NUM_3].Init.Mode                = DMA_CIRCULAR;
    hdma_rx[DRV_UART_NUM_3].Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx[DRV_UART_NUM_3]);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_rx[DRV_UART_NUM_3]);

    // TELL DMA ISR handler the handle to use during ISRs...
    SetDMA1StreamHandlerHandle(1, &hdma_rx[DRV_UART_NUM_3], false, NULL);

    /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

#ifdef DRV_UART_RX_DMA_ONLY
    if(is_uart_mode[DRV_UART_NUM_3] == DRV_UART_DMA_MODE)
#endif
    {
      hdma_tx[DRV_UART_NUM_3].Instance                 = DMA1_Stream3;
      hdma_tx[DRV_UART_NUM_3].Init.Channel             = DMA_CHANNEL_4;
      hdma_tx[DRV_UART_NUM_3].Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_tx[DRV_UART_NUM_3].Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_tx[DRV_UART_NUM_3].Init.MemInc              = DMA_MINC_ENABLE;
      hdma_tx[DRV_UART_NUM_3].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_tx[DRV_UART_NUM_3].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma_tx[DRV_UART_NUM_3].Init.Mode                = DMA_NORMAL;
      hdma_tx[DRV_UART_NUM_3].Init.Priority            = DMA_PRIORITY_MEDIUM;

      HAL_DMA_Init(&hdma_tx[DRV_UART_NUM_3]);

      /* Associate the initialized DMA handle to the the UART handle */
      __HAL_LINKDMA(huart, hdmatx, hdma_tx[DRV_UART_NUM_3]);

      // TELL DMA ISR handler the handle to use during ISRs...
      is_uart_write_dma_mode[DRV_UART_NUM_3] = SetDMA1StreamHandlerHandle(3, &hdma_tx[DRV_UART_NUM_3], false, &HAL_UART_LoseDMAHandler);
      if (is_uart_write_dma_mode[DRV_UART_NUM_3])
      {
        /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
      }
      else 
      {
        //vcp_printf(" Serial3 TX not DMA\n");
      }
    }

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (USART3_IRQn);
  }
  else if(huart->Instance==UART8) // // UART_NUM_4
  {
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
    RCC_PeriphCLKInitStruct.Uart8ClockSelection  = RCC_UART8CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

#ifdef DRV_UART_RX_DMA_ONLY
    if(is_uart_mode[DRV_UART_NUM_4] == DRV_UART_DMA_MODE) 
#endif
    {
      // DMA Setup
      /* Configure the DMA handler for reception process */
      __HAL_RCC_DMA1_CLK_ENABLE();
      hdma_rx[DRV_UART_NUM_4].Instance                 = DMA1_Stream6;
      hdma_rx[DRV_UART_NUM_4].Init.Channel             = DMA_CHANNEL_5;
      hdma_rx[DRV_UART_NUM_4].Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma_rx[DRV_UART_NUM_4].Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_rx[DRV_UART_NUM_4].Init.MemInc              = DMA_MINC_ENABLE;
      hdma_rx[DRV_UART_NUM_4].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_rx[DRV_UART_NUM_4].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma_rx[DRV_UART_NUM_4].Init.Mode                = DMA_CIRCULAR;
      hdma_rx[DRV_UART_NUM_4].Init.Priority            = DMA_PRIORITY_HIGH;

      HAL_DMA_Init(&hdma_rx[DRV_UART_NUM_4]);

      /* Associate the initialized DMA handle to the the UART handle */
      __HAL_LINKDMA(huart, hdmarx, hdma_rx[DRV_UART_NUM_4]);

      // TELL DMA ISR handler the handle to use during ISRs...
      SetDMA1StreamHandlerHandle(6, &hdma_rx[DRV_UART_NUM_4], true, NULL);
 
      /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
      HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

#ifndef DRV_UART_RX_DMA_ONLY
      if(is_uart_mode[DRV_UART_NUM_4] == DRV_UART_DMA_MODE) 
#endif
      {
        hdma_tx[DRV_UART_NUM_4].Instance                 = DMA1_Stream0;
        hdma_tx[DRV_UART_NUM_4].Init.Channel             = DMA_CHANNEL_5;
        hdma_tx[DRV_UART_NUM_4].Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tx[DRV_UART_NUM_4].Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tx[DRV_UART_NUM_4].Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tx[DRV_UART_NUM_4].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_4].Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_tx[DRV_UART_NUM_4].Init.Mode                = DMA_NORMAL;
        hdma_tx[DRV_UART_NUM_4].Init.Priority            = DMA_PRIORITY_MEDIUM;

        HAL_DMA_Init(&hdma_tx[DRV_UART_NUM_4]);

        /* Associate the initialized DMA handle to the the UART handle */
        __HAL_LINKDMA(huart, hdmatx, hdma_tx[DRV_UART_NUM_4]);

        // TELL DMA ISR handler the handle to use during ISRs...
        is_uart_write_dma_mode[DRV_UART_NUM_4] = SetDMA1StreamHandlerHandle(0, &hdma_tx[DRV_UART_NUM_4], false, &HAL_UART_LoseDMAHandler);
        if (is_uart_write_dma_mode[DRV_UART_NUM_4])
        {
          /* NVIC configuration for DMA transfer complete interrupt (USART3_RX) */
          HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
          HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
        }
        else 
        {
          //vcp_printf(" Serial4 TX not DMA\n");
        }
      }

    }



    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ  (UART8_IRQn);

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
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
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
  else if(huart->Instance==UART8)
  {
    __UART8_FORCE_RESET();
    __UART8_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  }

}
