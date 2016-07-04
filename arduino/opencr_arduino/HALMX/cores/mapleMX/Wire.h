/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

/*
 * Arduino srl - www.arduino.org
 * 2016 Jun 9: Edited Francesco Alessi (alfran) - francesco@arduino.org
 */

/**
 * Ported to HALMX - www.stm32duino.com
 * 2016 Jun 20: Modified by Vassilis Serasidis - avrsite@yahoo.gr
 */
 
#ifndef TwoWire_h
#define TwoWire_h

#include "hal.h"
#include <inttypes.h>
#include "Stream.h"
#include "chip.h"

#define BUFFER_LENGTH 32

class TwoWire : public Stream
{
  private:
    uint8_t txAddress;
    uint8_t txBuffer[BUFFER_LENGTH];
    uint8_t txBufferIndex;
    uint8_t txBufferLength;

    uint8_t transmitting;
    void (*user_onRequest)(void);
    // I2C address
    uint8_t defaultAddress;

  int i2c_master_start();
    int i2c_master_stop();
    int i2c_master_read(uint8_t address, char *data, uint8_t length, uint8_t stop);
    int i2c_master_write(int address, const char *data, int length, int stop);
    int i2c_master_byte_read(int *value, int last);
    int i2c_master_byte_write(int data);
    int i2c_slave_write(const char *data, int length);
  void enableInterrupt(void);
  void disableInterrupt(void);
  public:
  TwoWire(I2C_TypeDef *twi);
    void begin();
    void begin(uint8_t);
    void begin(int);
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

  void handleInterrupt(uint8_t TransferDirection, uint16_t AddrMatchCode);
  /* I2C handler declaration */
    I2C_HandleTypeDef I2cHandle;
  uint8_t rxBuffer[BUFFER_LENGTH];
    volatile uint8_t rxBufferIndex;
    volatile uint8_t rxBufferLength;
  void (*user_onReceive)(int);
};

#if BOARD_NR_I2C > 0
extern TwoWire Wire;
#endif
#if BOARD_NR_I2C > 1
extern TwoWire Wire1;
#endif
#if BOARD_NR_I2C > 2
extern TwoWire Wire2;
#endif

#endif
