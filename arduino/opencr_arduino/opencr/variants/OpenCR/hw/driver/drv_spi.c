/*
 *  drv_spi.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_spi.h"
#include "variant.h"


#define SPI_MAX_CH              2
#define SPI_TX_DMA_MAX_LENGTH   0xEFFF



typedef struct
{
  bool use;
  bool init;
  bool tx_done;
  uint8_t *p_tx_buf;
  uint8_t *p_tx_buf_next;
  uint32_t tx_length_next;
} spi_dma_t;


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

static DMA_HandleTypeDef hdma2_tx;

volatile spi_dma_t spi_dma[SPI_MAX_CH];






int drv_spi_init()
{
  uint8_t i;


  hspi1.Instance                = SPI1;
  hspi1.Init.Mode               = SPI_MODE_MASTER;
  hspi1.Init.Direction          = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize           = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity        = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase           = SPI_PHASE_1EDGE;
  hspi1.Init.NSS                = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode             = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial      = 10;
  //HAL_SPI_Init(&hspi1);

  hspi2.Instance                = SPI2;
  hspi2.Init.Mode               = SPI_MODE_MASTER;
  hspi2.Init.Direction          = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize           = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity        = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase           = SPI_PHASE_1EDGE;
  hspi2.Init.NSS                = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode             = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial      = 10;
  //HAL_SPI_Init(&hspi2);


  for(i=0; i<SPI_MAX_CH; i++)
  {
    spi_dma[i].p_tx_buf         = NULL;
    spi_dma[i].p_tx_buf_next    = NULL;
    spi_dma[i].use              = false;
    spi_dma[i].init             = false;
    spi_dma[i].tx_done          = false;
    spi_dma[i].tx_length_next   = 0;
  }

  return 0;
}


void drv_spi_enable_dma(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  }
  if(hspi->Instance==SPI2)
  {
    spi_dma[1].use = true;
  }
}


uint8_t drv_spi_is_dma_tx_done(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
    if(spi_dma[0].use != true) return true;

    return spi_dma[0].tx_done;
  }
  if(hspi->Instance==SPI2)
  {
    if(spi_dma[1].use != true) return true;

    return spi_dma[1].tx_done;
  }

  return true;
}


void drv_spi_start_dma_tx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint32_t length)
{
  if(hspi->Instance==SPI1)
  {
  }
  if(hspi->Instance==SPI2)
  {
    if(spi_dma[1].use != true) return;

    if(length > SPI_TX_DMA_MAX_LENGTH)
    {
      spi_dma[1].tx_done       = false;
      spi_dma[1].tx_length_next= length - SPI_TX_DMA_MAX_LENGTH;
      spi_dma[1].p_tx_buf      =  p_buf;
      spi_dma[1].p_tx_buf_next = &p_buf[SPI_TX_DMA_MAX_LENGTH];

      HAL_SPI_Transmit_DMA(hspi, spi_dma[1].p_tx_buf, SPI_TX_DMA_MAX_LENGTH);
    }
    else
    {
      spi_dma[1].tx_done       = false;
      spi_dma[1].tx_length_next= 0;
      spi_dma[1].p_tx_buf      = p_buf;
      spi_dma[1].p_tx_buf_next = NULL;

      HAL_SPI_Transmit_DMA(hspi, spi_dma[1].p_tx_buf, length);
    }
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  volatile uint16_t length;


  if(hspi->Instance==SPI2)
  {
    if(spi_dma[1].tx_length_next > 0)
    {
      spi_dma[1].p_tx_buf = spi_dma[1].p_tx_buf_next;

      if(spi_dma[1].tx_length_next > SPI_TX_DMA_MAX_LENGTH)
      {
        length = SPI_TX_DMA_MAX_LENGTH;
        spi_dma[1].tx_length_next = spi_dma[1].tx_length_next - SPI_TX_DMA_MAX_LENGTH;
        spi_dma[1].p_tx_buf_next = &spi_dma[1].p_tx_buf[SPI_TX_DMA_MAX_LENGTH];
      }
      else
      {
        length = spi_dma[1].tx_length_next;
        spi_dma[1].tx_length_next = 0;
        spi_dma[1].p_tx_buf_next = NULL;
      }
      HAL_SPI_Transmit_DMA(hspi, spi_dma[1].p_tx_buf, length);
    }
    else
    {
      spi_dma[1].tx_done = true;
    }

  }
}


// SPIx_DMA_TX_IRQHandler(void)
//
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmatx);
}


void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;


  if(hspi->Instance==SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if(hspi->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    if(spi_dma[1].use == true && spi_dma[1].init == false)
    {
      spi_dma[1].init = true;

      bsp_mpu_config();

      __HAL_RCC_DMA1_CLK_ENABLE();

      /*##-3- Configure the DMA ##################################################*/
      /* Configure the DMA handler for Transmission process */
      hdma2_tx.Instance                 = DMA1_Stream4;
      hdma2_tx.Init.Channel             = DMA_CHANNEL_0;
      hdma2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma2_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma2_tx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma2_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
      hdma2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma2_tx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma2_tx.Init.Mode                = DMA_NORMAL;
      hdma2_tx.Init.Priority            = DMA_PRIORITY_LOW;

      HAL_DMA_Init(&hdma2_tx);

      /* Associate the initialized DMA handle to the the SPI handle */
      __HAL_LINKDMA(hspi, hdmatx, hdma2_tx);


      HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
      HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    }
  }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  }

  if(hspi->Instance==SPI2)
  {
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    __HAL_RCC_SPI2_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }

}
