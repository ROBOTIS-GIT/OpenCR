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
SPIClass SPI    (SPI2);
SPIClass SPI_IMU(SPI1);



void SPISettings::init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
  if (clock >= 50000000 / 2) {
    clockDiv = SPI_CLOCK_DIV2;
  } else if (clock >= 50000000 / 4) {
    clockDiv = SPI_CLOCK_DIV4;
  } else if (clock >= 50000000 / 8) {
    clockDiv = SPI_CLOCK_DIV8;
  } else if (clock >= 50000000 / 16) {
    clockDiv = SPI_CLOCK_DIV16;
  } else if (clock >= 50000000 / 32) {
    clockDiv = SPI_CLOCK_DIV32;
  } else if (clock >= 50000000 / 64) {
    clockDiv = SPI_CLOCK_DIV64;
  } else {
    clockDiv = SPI_CLOCK_DIV64;
  }

  _bitOrder = bitOrder;
  _dataMode = dataMode;
}


SPIClass::SPIClass(SPI_TypeDef *spiPort) {
  _spiPort = spiPort;

  if(spiPort == SPI1)
    _hspi = &hspi1;
  if(spiPort == SPI2)
    _hspi = &hspi2;
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
  }
}


void SPIClass::begin(void) {
  init();
}


void SPIClass::init(void){

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


  if(_spiPort == SPI1){
    __HAL_RCC_SPI1_CLK_ENABLE();
  }

  if(_spiPort == SPI2){
    __HAL_RCC_SPI2_CLK_ENABLE();
  }
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

void SPIClass::transfer(void *buf, size_t count) {
  HAL_SPI_Transmit(_hspi, (uint8_t *)buf, count, 0xffff);
}


void SPIClass::setBitOrder(uint8_t bitOrder) {
    if (bitOrder == 1)
      _hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    else
      _hspi->Init.FirstBit = SPI_FIRSTBIT_LSB;
    HAL_SPI_Init(_hspi);
}

void SPIClass::setClockDivider(uint8_t clockDiv) {
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

void SPIClass::setDataMode(uint8_t dataMode){

  switch( dataMode )
  {
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
