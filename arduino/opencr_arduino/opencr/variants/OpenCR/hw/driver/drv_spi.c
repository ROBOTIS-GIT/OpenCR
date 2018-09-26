/*
 *  drv_spi.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */

#include "drv_spi.h"
#include "variant.h"
#include "dma_stream_handlers.h"


#define SPI_MAX_CH              3
#define SPI_TX_DMA_MAX_LENGTH   0xEFFF



typedef struct
{
  bool    use;
  bool    init;
  uint32_t length_left;
  bool    transfer_done;
  // TX state variables
  uint8_t *p_tx_buf;
  // RX state variables - Maybe some can be combined with TX
  uint8_t *p_rx_buf;
  DrvSPIDMACallback dma_callback;    // Optional function to call at DMA completion
} spi_dma_t;


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

static DMA_HandleTypeDef hdma2_tx;
static DMA_HandleTypeDef hdma2_rx;
static DMA_HandleTypeDef hdma4_tx;
static DMA_HandleTypeDef hdma4_rx;

volatile spi_dma_t spi_dma[SPI_MAX_CH];


inline volatile spi_dma_t *drv_map_haspi_to_spi_dma(SPI_HandleTypeDef* hspi) 
{
  if (hspi->Instance == SPI1) return &spi_dma[0];
  if (hspi->Instance == SPI2) return &spi_dma[1];
  if (hspi->Instance == SPI4) return &spi_dma[2];
  return NULL;
}




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

  hspi4.Instance                = SPI4;
  hspi4.Init.Mode               = SPI_MODE_MASTER;
  hspi4.Init.Direction          = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize           = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity        = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase           = SPI_PHASE_1EDGE;
  hspi4.Init.NSS                = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode             = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial      = 10;
  //HAL_SPI_Init(&hspi4);


  for(i=0; i<SPI_MAX_CH; i++)
  {
    spi_dma[i].p_tx_buf         = NULL;
    spi_dma[i].p_rx_buf         = NULL;
    spi_dma[i].use              = false;
    spi_dma[i].init             = false;
    spi_dma[i].transfer_done    = false;
    spi_dma[i].length_left      = 0;
    spi_dma[i].dma_callback     = NULL;
  }

  return 0;
}


void drv_spi_enable_dma(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  }
  else if(hspi->Instance==SPI2)
  {
    spi_dma[1].use = true;
  }
  else if(hspi->Instance==SPI4)
  {
    spi_dma[2].use = true;
  }
}

bool drv_spi_dma_enabled(SPI_HandleTypeDef* hspi) 
{
 volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);
 return pspi_dma->use;
}

//=================================================================
// Support for DMA TX Only
//=================================================================

uint8_t drv_spi_is_dma_tx_done(SPI_HandleTypeDef* hspi)
{
  volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);
  if (!pspi_dma || (pspi_dma->use != true)) return true;  
  return pspi_dma->transfer_done;
}


void drv_spi_start_dma_tx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint32_t length, DrvSPIDMACallback dma_callback)
{
 volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);

  if ((pspi_dma == NULL) || (pspi_dma->use != true)) return ;

  pspi_dma->transfer_done       = false;
  pspi_dma->length_left     = length;
  pspi_dma->p_tx_buf        =  p_buf;
  pspi_dma->p_rx_buf        = NULL;   // this will funnel through txrx is set to NULL
  pspi_dma->transfer_done   = false;
  pspi_dma->dma_callback    = dma_callback;
  
  if(length > SPI_TX_DMA_MAX_LENGTH)
    length = SPI_TX_DMA_MAX_LENGTH;

  pspi_dma->length_left -= length;

  HAL_SPI_TransmitReceive_DMA(hspi, pspi_dma->p_tx_buf, NULL, length);
  //HAL_SPI_Transmit_DMA(hspi, pspi_dma->p_tx_buf, SPI_TX_DMA_MAX_LENGTH);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);    // digitalWrite(0, HIGH);
  volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);
  volatile uint32_t length;
  if (pspi_dma && (hspi->Instance != SPI1))
  {
    length = pspi_dma->length_left;

    if(length > 0)
    {
      pspi_dma->p_tx_buf += SPI_TX_DMA_MAX_LENGTH;
      if (length > SPI_TX_DMA_MAX_LENGTH)
      {
        length = SPI_TX_DMA_MAX_LENGTH;
      }
      pspi_dma->length_left -= length;
      //HAL_SPI_Transmit_DMA(hspi, pspi_dma->p_tx_buf, length);
      HAL_SPI_TransmitReceive_DMA(hspi, pspi_dma->p_tx_buf, NULL, length);
    }
    else
    {
      pspi_dma->transfer_done = true;
      if (pspi_dma->dma_callback) 
      {
        (*pspi_dma->dma_callback)(hspi);
      }
    }

  }
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);    // digitalWrite(0, HIGH);
}


// SPIx_DMA_TX_IRQHandler(void)
//
#if 0
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmatx);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi4.hdmatx);
}


// SPIx_DMA_RX_IRQHandler(void)
//
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmarx);
}

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi4.hdmarx);
}
#endif

//=================================================================
// Support for DMA Transfer
//=================================================================
uint8_t drv_spi_is_dma_txrx_done(SPI_HandleTypeDef* hspi)
{
  volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);
  if (!pspi_dma || (pspi_dma->use != true)) return true;  
  return pspi_dma->transfer_done;
}


void drv_spi_start_dma_txrx(SPI_HandleTypeDef* hspi, uint8_t *p_buf, uint8_t *p_rxbuf, uint32_t length, DrvSPIDMACallback dma_callback)
{
 volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);

  if ((pspi_dma == NULL) || (pspi_dma->use != true)) return ;

  // Assume TX and RX max length are the same
  pspi_dma->p_tx_buf        =  p_buf;
  pspi_dma->p_rx_buf        =  p_rxbuf;
  pspi_dma->length_left     = length;
  pspi_dma->transfer_done   = false;
  pspi_dma->dma_callback    = dma_callback;


  if (length > SPI_TX_DMA_MAX_LENGTH) 
  {
    length = SPI_TX_DMA_MAX_LENGTH;
  }
  pspi_dma->length_left -= length;
  HAL_SPI_TransmitReceive_DMA(hspi, pspi_dma->p_tx_buf, pspi_dma->p_rx_buf, length);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  volatile spi_dma_t *pspi_dma = drv_map_haspi_to_spi_dma(hspi);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);    // digitalWrite(0, HIGH);
  if (pspi_dma && (hspi->Instance != SPI1))
  {
    volatile uint32_t length = pspi_dma->length_left;
    if(length > 0)
    {
      if (pspi_dma->p_rx_buf)
      {
        pspi_dma->p_rx_buf += SPI_TX_DMA_MAX_LENGTH;
      }
      if (pspi_dma->p_tx_buf)
      {
        pspi_dma->p_tx_buf += SPI_TX_DMA_MAX_LENGTH;
      }

      if (length > SPI_TX_DMA_MAX_LENGTH)
      {
        length = SPI_TX_DMA_MAX_LENGTH;
      }
      pspi_dma->length_left -= length;
      //vcp_printf("TxRxCB: %x %x %d\n", (uint32_t)pspi_dma->p_tx_buf, (uint32_t)pspi_dma->p_rx_buf, length);
      HAL_SPI_TransmitReceive_DMA(hspi, pspi_dma->p_tx_buf, pspi_dma->p_rx_buf, length);
    }
    else
    {
      pspi_dma->transfer_done = true;
      if (pspi_dma->dma_callback) 
      {
        (*pspi_dma->dma_callback)(hspi);
      }
    }

  }
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);    // digitalWrite(0, LOW);
}



//=================================================================
// Init HAL SPI
//=================================================================
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;


  // BUGBUG:: Would be nice to more table drive this!

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
    // TELL DMA ISR handler the handle to use during ISRs...
    SetDMA1StreamHandlerHandle(4, &hdma2_tx, true, NULL);


      HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
      HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

      /* Configure the DMA handler for receive process */
      hdma2_rx.Instance                 = DMA1_Stream3;
      hdma2_rx.Init.Channel             = DMA_CHANNEL_0;
      hdma2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma2_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma2_rx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma2_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
      hdma2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma2_rx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma2_rx.Init.Mode                = DMA_NORMAL;
      hdma2_rx.Init.Priority            = DMA_PRIORITY_LOW;

      HAL_DMA_Init(&hdma2_rx);

      /* Associate the initialized DMA handle to the the SPI handle */
      __HAL_LINKDMA(hspi, hdmarx, hdma2_rx);

    // TELL DMA ISR handler the handle to use during ISRs...
    SetDMA1StreamHandlerHandle(3, &hdma2_rx, true, NULL);

      HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 1);
      HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    }
  }

  if(hspi->Instance==SPI4)
  {
    __HAL_RCC_SPI4_CLK_ENABLE();
    // SCK - Arduino pin 58
    GPIO_InitStruct.Pin       = GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  59 */
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  60 */
    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    if(spi_dma[2].use == true && spi_dma[2].init == false)
    {
      spi_dma[2].init = true;

      bsp_mpu_config();

      __HAL_RCC_DMA2_CLK_ENABLE();

      /* Configure the DMA handler for Transmission process */
      hdma4_tx.Instance                 = DMA2_Stream1;
      hdma4_tx.Init.Channel             = DMA_CHANNEL_4;
      hdma4_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma4_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma4_tx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma4_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
      hdma4_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma4_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma4_tx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma4_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma4_tx.Init.Mode                = DMA_NORMAL;
      hdma4_tx.Init.Priority            = DMA_PRIORITY_LOW;

      HAL_DMA_Init(&hdma4_tx);

      /* Associate the initialized DMA handle to the the SPI handle */
      __HAL_LINKDMA(hspi, hdmatx, hdma4_tx);

      // TELL DMA ISR handler the handle to use during ISRs...
      SetDMA2StreamHandlerHandle(1, &hdma4_tx, true, NULL);


      HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
      HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

      /* Configure the DMA handler for receive process */
      hdma4_rx.Instance                 = DMA2_Stream0;
      hdma4_rx.Init.Channel             = DMA_CHANNEL_4;
      hdma4_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma4_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma4_rx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma4_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
      hdma4_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma4_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma4_rx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma4_rx.Init.Mode                = DMA_NORMAL;
      hdma4_rx.Init.Priority            = DMA_PRIORITY_LOW;

      HAL_DMA_Init(&hdma4_rx);

      /* Associate the initialized DMA handle to the the SPI handle */
      __HAL_LINKDMA(hspi, hdmarx, hdma4_rx);

      // TELL DMA ISR handler the handle to use during ISRs...
      SetDMA2StreamHandlerHandle(0, &hdma4_rx, true, NULL);

      HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 1);
      HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  else if(hspi->Instance==SPI2)
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
  else if(hspi->Instance==SPI4)
  {
    __HAL_RCC_SPI4_FORCE_RESET();
    __HAL_RCC_SPI4_RELEASE_RESET();
    __HAL_RCC_SPI4_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_13);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_14);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(SPI4_IRQn);
  }
}
