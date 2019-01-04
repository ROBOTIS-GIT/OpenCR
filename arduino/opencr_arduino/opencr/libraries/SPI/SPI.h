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


#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <chip.h>
#include "variant.h"

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1
// SPI_HAS_TRANSFER_BUF - is defined to signify that this library supports
// a version of transfer which allows you to pass in both TX and RX buffer
// pointers, either of which could be NULL
#define SPI_HAS_TRANSFER_BUF 1

#if defined(__has_include) && __has_include(<EventResponder.h>)
#define SPI_HAS_TRANSFER_ASYNC 1
#include <EventResponder.h>
#endif



#define SPI_CLOCK_DIV4      0x00
#define SPI_CLOCK_DIV16     0x01
#define SPI_CLOCK_DIV64     0x02
#define SPI_CLOCK_DIV128    0x03
#define SPI_CLOCK_DIV2      0x04
#define SPI_CLOCK_DIV8      0x05
#define SPI_CLOCK_DIV32     0x06
#define SPI_CLOCK_DIV256    0x07

#define SPI_MODE0           0
#define SPI_MODE1           1
#define SPI_MODE2           2
#define SPI_MODE3           3


#ifdef USE_SPI1
extern SPI_HandleTypeDef hspi1;
#endif
#ifdef USE_SPI2
extern SPI_HandleTypeDef hspi2;
#endif
#ifdef USE_SPI4
extern SPI_HandleTypeDef hspi4;
#endif


class SPISettings {
public:
  SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
  {
    if (__builtin_constant_p(clock)) {
      init_Inline(clock, bitOrder, dataMode);
    } else {
      init_NotInline(clock, bitOrder, dataMode);
    }
  }
  SPISettings()
  {
    init_Inline(4000000, MSBFIRST, SPI_MODE0);
  }
private:
  void init_NotInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
    init_Inline(clock, bitOrder, dataMode);
  }


//The devices feature up to six SPIs in slave and master modes in full-duplex and simplex communication modes. 
// SPI1, SPI4, SPI5, and SPI6 can communicate at up to 50 Mbits/s, SPI2 and SPI3 can communicate at up to 25 Mbit/s. 
  void init_Inline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) 
    __attribute__((__always_inline__))  {
      _clock = clock;
      _bitOrder = bitOrder;
      _dataMode = dataMode;
  }

  //uint8_t _clockDiv;
  uint32_t _clock;
  uint8_t _bitOrder;
  uint8_t _dataMode;

  friend class SPIClass;
};


class SPIClass {
  public:
    SPIClass(SPI_TypeDef *spiPort, uint32_t spi_clock);
    SPIClass(uint8_t spiPort);
    void begin(void);
    void beginFast(void);

    void beginTransaction(SPISettings settings)
    {
      if (settings._clock != _clock) {
        setClock(settings._clock);
      }
      if (settings._bitOrder != _bitOrder) {
        setBitOrder(settings._bitOrder);
      }
      if (settings._dataMode != _dataMode) {
        setDataMode(settings._dataMode);
      }
    }
    void endTransaction(void)
    {
    }

    // Disable the SPI bus
    static void end();

    uint8_t transfer(uint8_t _data) const;
    uint16_t transfer16(uint16_t data);
//    void transfer(void *buf, size_t count);

    void inline transfer(void *buf, size_t count) {transfer(buf, buf, count);}
    //void setTransferWriteFill(uint8_t ch ) {_transferWriteFill = ch;}
    void transfer(const void * buf, void * retbuf, size_t count);


    // Write only functions similar to ESP32 and the like
    void write(uint8_t data);
    void write16(uint16_t data);
    void write32(uint32_t data);
    void writeBytes(uint8_t * data, uint32_t size);

    void inline writeFast(void *buf, size_t count) {transfer(buf, NULL, count);}
    void writePixels(const void * data, uint32_t size);//ili9341 compatible


    void setBitOrder(uint8_t bitOrder);
    void setClockDivider(uint8_t clockDiv);
    void setClock(uint32_t clock);
    void setDataMode(uint8_t dataMode);

#ifdef SPI_HAS_TRANSFER_ASYNC
  bool transfer(const void *txBuffer, void *rxBuffer, size_t count,  EventResponderRef  event_responder);
#endif

  enum DMAState {DMA_NOTINITIALIZED=0, DMA_IDLE, DMA_ACTIVE, DMA_COMPETED};
  private:
    //uint32_t _Mode;
    uint32_t _Direction;
    uint32_t _DataSize;
    uint32_t _CLKPolarity;
    uint32_t _CLKPhase;
    uint32_t _NSS;
    uint32_t _BaudRatePrescaler;
    uint32_t _FirstBit;
    uint32_t _TIMode;
    uint32_t _CRCCalculation;
    uint32_t _CRCPolynomial;
    // Keep track of values we set for transactions 
    uint32_t _clock;
    uint8_t _bitOrder;
    uint8_t _dataMode;
    uint32_t _spi_clock;   

    SPI_HandleTypeDef *_hspi;
    SPI_TypeDef *_spiPort;
#ifdef SPI_HAS_TRANSFER_ASYNC
    uint8_t  _dma_state;
    EventResponder *_dma_event_responder;
static void dmaCallback(SPI_HandleTypeDef* hspi);
    void processDMACallback();
#endif
    void init(void);

};

extern SPIClass SPI;
extern SPIClass SPI_IMU;
extern SPIClass SPI_EXT;

#endif
