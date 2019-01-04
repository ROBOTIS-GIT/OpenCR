/****************************************************************************
 *
 * SPI Master library for Arduino STM32 + HAL + CubeMX (HALMX).
 *
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 * Home: http://www.serasidis.gr
 * email: avrsite@yahoo.gr
 *
 * Arduino_STM32 forum: http://www.stm32duino.com
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 ****************************************************************************/


#include <SPI.h>
#include <chip.h>



/* Create an SPIClass instance */
SPIClass SPI    (SPI2, 50000000);
SPIClass SPI_IMU(SPI1, 100000000);
SPIClass SPI_EXT(SPI4, 100000000);


SPIClass::SPIClass(SPI_TypeDef *spiPort, uint32_t spi_clock) {
  _spiPort = spiPort;

  if(spiPort == SPI1)
    _hspi = &hspi1;
  else if(spiPort == SPI2)
    _hspi = &hspi2;
  else if(spiPort == SPI4)
    _hspi = &hspi4;
  _spi_clock = spi_clock;
}

/**
 * This constructor is written for backward compatibility with Arduino_STM32
 * @Usage example: SPIClass my_spi(2) for using the SPI2 interface
 */
SPIClass::SPIClass(uint8_t spiPort){
  switch(spiPort){
    case 1:
      _spiPort = SPI1;
      _hspi = &hspi1;
    break;

    case 2:
      _spiPort = SPI2;
      _hspi = &hspi2;
    break;
    case 4:
      _spiPort = SPI4;
      _hspi = &hspi4;
    break;
  }
}


void SPIClass::begin(void) {
  init();
}

void SPIClass::beginFast(void) {
  drv_spi_enable_dma(_hspi);
  init();
}

void SPIClass::init(void){
  // Keep track of transaction logical values.
  //_clockDiv = SPI_CLOCK_DIV16;
  _bitOrder = MSBFIRST;
  _dataMode = SPI_MODE0;
  _dma_state = DMA_NOTINITIALIZED;
  _dma_event_responder   = NULL;
  _hspi->Instance               = _spiPort;
  _hspi->Init.Mode              = SPI_MODE_MASTER;
  _hspi->Init.Direction         = SPI_DIRECTION_2LINES;
  _hspi->Init.DataSize          = SPI_DATASIZE_8BIT;
  _hspi->Init.CLKPolarity       = SPI_POLARITY_LOW;
  _hspi->Init.CLKPhase          = SPI_PHASE_1EDGE;
  _hspi->Init.NSS               = SPI_NSS_SOFT;
  _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 4.5 Mbit
  _hspi->Init.FirstBit          = SPI_FIRSTBIT_MSB;
  _hspi->Init.TIMode            = SPI_TIMODE_DISABLE;
  _hspi->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  _hspi->Init.CRCPolynomial     = 10;
  HAL_SPI_Init(_hspi);
}

void SPIClass::end()
{
  
}

uint8_t SPIClass::transfer(uint8_t data) const{
  uint8_t ret;
  HAL_SPI_TransmitReceive(_hspi, &data, &ret, 1, 0xffff);
	return ret;
}

uint16_t SPIClass::transfer16(uint16_t data) {
  uint8_t tBuf[2];
  uint8_t rBuf[2];
  uint16_t ret;
  tBuf[1] = (uint8_t)data;
  tBuf[0] = (uint8_t)(data>>8);
  HAL_SPI_TransmitReceive(_hspi, (uint8_t *)&tBuf, (uint8_t *)&rBuf, 2, 0xffff);

  ret = rBuf[0];
  ret <<= 8;
  ret += rBuf[1];

  return ret;
}


void SPIClass::transfer(const void * buf, void * retbuf, size_t count) {
  if ((count == 0) || ((buf == NULL) && (retbuf == NULL))) return;    // nothing to do

//  bool dma_enabled = drv_spi_dma_enabled(_hspi);
  HAL_StatusTypeDef status;
  if (retbuf == NULL) { 
    // write only transfer
    status = HAL_SPI_Transmit(_hspi, (uint8_t *)buf, count, 0xffff);
  } else if (buf == NULL) {
    // Read only buffer
    status = HAL_SPI_Receive(_hspi, (uint8_t*)retbuf, count, 0xffff);
  } else {
    // standard Read/write buffer transfer
    // start off without DMA support
    status = HAL_SPI_TransmitReceive(_hspi, (uint8_t *)buf, (uint8_t*)retbuf, count, 0xffff);
  }
  if (status != HAL_OK) 
  {
    Serial.print("transfer status: ");
    Serial.println((int)status, DEC);
  }
}


// Write only functions many used by Adafruit libraries.

void SPIClass::write(uint8_t data) {
  HAL_SPI_Transmit(_hspi, &data, 1, 0xffff);
}

void SPIClass::write16(uint16_t data) {
  uint8_t tBuf[2];
  tBuf[0] = (uint8_t)(data>>8);
  tBuf[1] = (uint8_t)data;
  HAL_SPI_Transmit(_hspi, tBuf, 2, 0xffff);
}

void SPIClass::write32(uint32_t data) {
  uint8_t tBuf[4];
  tBuf[0] = (uint8_t)(data>>24);
  tBuf[1] = (uint8_t)(data>>16);
  tBuf[2] = (uint8_t)(data>>8);
  tBuf[3] = (uint8_t)data;
  HAL_SPI_Transmit(_hspi, tBuf, 4, 0xffff);
}

void SPIClass::writeBytes(uint8_t * data, uint32_t size) {
  HAL_SPI_Transmit(_hspi, data, size, 0xffff);
}

void SPIClass::writePixels(const void * data, uint32_t size) { //ili9341 compatible
    // First pass a hack! need to reverse bytes... Use internal buffer.. 
    uint8_t tBuf[64];
    uint16_t *pixels = (uint16_t *)data;

    // size is the number of bytes. 
    while (size) {
      uint8_t *pb = tBuf;

      uint32_t cb = (size > 64)? 64 : size;

      for (uint32_t i = 0; i < cb; i += 2) {
        *pb++ = *pixels >> 8;
        *pb++ = (uint8_t)*pixels;
        pixels++;
      }
      HAL_SPI_Transmit(_hspi, tBuf, cb, 0xffff);
      size -= cb; 

    }
}



void SPIClass::setBitOrder(uint8_t bitOrder) {
  _bitOrder = bitOrder;
  if (bitOrder == 1)
    _hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  else
    _hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;
    HAL_SPI_Init(_hspi);
}


void SPIClass::setClockDivider(uint8_t clockDiv) {
  _clock = 0;   // clear out so we will set in setClock
  switch(clockDiv){
    case SPI_CLOCK_DIV2:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    break;
    case SPI_CLOCK_DIV4:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    break;
    case SPI_CLOCK_DIV8:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    break;
    case SPI_CLOCK_DIV16:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    break;
    case SPI_CLOCK_DIV32:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    break;
    case SPI_CLOCK_DIV64:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    break;
    case SPI_CLOCK_DIV128:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    break;
    case SPI_CLOCK_DIV256:
      _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    break;
  }
  HAL_SPI_Init(_hspi);
}

void SPIClass::setClock(uint32_t clock) {
  _clock = clock; // remember our new clock  
  if (clock >= _spi_clock / 2) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if (clock >= _spi_clock / 4) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (clock >= _spi_clock / 8) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (clock >= _spi_clock / 16) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (clock >= _spi_clock / 32) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (clock >= _spi_clock / 64) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (clock >= _spi_clock / 128) {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else {
    _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;  // Our slowest mode
  }
  HAL_SPI_Init(_hspi);
}

void SPIClass::setDataMode(uint8_t dataMode){

  switch( dataMode )
  {
    _dataMode = dataMode;
    // CPOL=0, CPHA=0
    case SPI_MODE0:
      _hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
      _hspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(_hspi);
      break;

    // CPOL=0, CPHA=1
    case SPI_MODE1:
      _hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
      _hspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(_hspi);
      break;

    // CPOL=1, CPHA=0
    case SPI_MODE2:
      _hspi->Init.CLKPolarity  = SPI_POLARITY_HIGH;
      _hspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(_hspi);
      break;

    // CPOL=1, CPHA=1
    case SPI_MODE3:
      _hspi->Init.CLKPolarity  = SPI_POLARITY_HIGH;
      _hspi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(_hspi);
      break;
  }
}
//=========================================================================
// Main Async Transfer function
//=========================================================================

bool SPIClass::transfer(const void *buf, void *retbuf, size_t count, EventResponderRef event_responder) {
//    Serial.println("Transfer with Event Call"); Serial.flush();
  if (_dma_state == DMA_ACTIVE)
    return false; // already active
  else if (_dma_state == DMA_NOTINITIALIZED)
  {
//    Serial.println("Before SPI enable DMA"); Serial.flush();
    drv_spi_enable_dma(_hspi);
    _dma_state = DMA_IDLE;
  }
//  Serial.println("Before Clear event");  Serial.flush();
  event_responder.clearEvent(); // Make sure it is not set yet
  if (count < 2) {
    // Use non-async version to simplify cases...
    transfer(buf, retbuf, count);
    event_responder.triggerEvent();
    return true;
  }

  if ((count == 0) || ((buf == NULL) && (retbuf == NULL))) return false;    // nothing to do

  _dma_event_responder = &event_responder;  // remember the event object
  //bool dma_enabled = drv_spi_dma_enabled(_hspi);
  // new version handles where buf or retbuf are null
  drv_spi_start_dma_txrx(_hspi, (uint8_t *)buf, (uint8_t *)retbuf, count, &dmaCallback);
  _dma_state = DMA_ACTIVE;
  return true;
}

void SPIClass::dmaCallback(SPI_HandleTypeDef* hspi)
{
  // Static function call from our DMA DRV code
  if (hspi == &hspi1) SPI_IMU.processDMACallback();
  else if (hspi == &hspi2) SPI.processDMACallback();
  else if (hspi == &hspi4) SPI_EXT.processDMACallback();

}

void SPIClass::processDMACallback()
{
  // We have been called back, that the DMA completed
  if (_dma_event_responder)
  {
    _dma_state = DMA_COMPETED;   // set back to 1 in case our call wants to start up dma again
    _dma_event_responder->triggerEvent();
  }

}
