/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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
 
extern "C"
{
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "hal.h"
}

#include "Wire.h"
#include "chip.h"

#define FLAG_TIMEOUT ((int)0x1000)
#define LONG_TIMEOUT ((int)0x8000)

// Constructors ////////////////////////////////////////////////////////////////
TwoWire::TwoWire(I2C_TypeDef *twi)
{
  I2cHandle.Instance = twi;
  memset(rxBuffer, 0, BUFFER_LENGTH);
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txAddress = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
  transmitting = 0;
  defaultAddress = 0x00;
}

// Public Methods //////////////////////////////////////////////////////////////

void TwoWire::begin(void)
{
}

void TwoWire::begin(uint8_t address)
{
  defaultAddress = (address << 1);
  begin();
}

void TwoWire::begin(int address)
{
  begin((uint8_t)address);
}

void TwoWire::setClock(uint32_t frequency)
{
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  uint8_t ret_val;


  return ret_val;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

//
//  Originally, 'endTransmission' was an f(void) function.
//  It has been modified to take one parameter indicating
//  whether or not a STOP should be performed on the bus.
//  Calling endTransmission(false) allows a sketch to
//  perform a repeated start.
//
//  WARNING: Nothing in the library keeps track of whether
//  the bus tenure has been properly ended with a STOP. It
//  is very possible to leave the bus in a hung state if
//  no call to endTransmission(true) is made. Some I2C
//  devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  // transmit buffer (blocking)
  disableInterrupt();

  int8_t ret = i2c_master_write((txAddress << 1), (const char *)txBuffer, txBufferLength, sendStop);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;

  enableInterrupt();

  return ret;
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(uint8_t data)
{
  if(transmitting){
    // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= BUFFER_LENGTH){
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;
  }else{
    // in slave send mode
  // transmit buffer (blocking)
    disableInterrupt();

    // reply to master
  i2c_slave_write((const char *)&data, 1);

  enableInterrupt();

  //HAL_I2C_EnableListen_IT(&I2cHandle);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  if(transmitting){
  // in master transmitter mode
    for(size_t i = 0; i < quantity; ++i){
      write(data[i]);
    }
  }else{
    // in slave send mode
    // reply to master
  disableInterrupt();

  i2c_slave_write((const char *)data, quantity);

  enableInterrupt();

  //HAL_I2C_EnableListen_IT(&I2cHandle);
  }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void)
{
  int value = -1;

  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::peek(void)
{
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::flush(void)
{
  // XXX: to be implemented.
}

// sets function called on slave write
void TwoWire::onReceive( void (*function)(int) )
{
  user_onReceive = function;
}

// sets function called on slave read
void TwoWire::onRequest( void (*function)(void) )
{
  user_onRequest = function;
}

int TwoWire::i2c_master_start()
{

    return 0;
}

int TwoWire::i2c_master_stop()
{

    return 0;
}

int TwoWire::i2c_master_read(uint8_t address, char *data, uint8_t length, uint8_t stop)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)I2cHandle.Instance;
    int timeout;
    int count;
    int value;
    int ret;


    return length;
}

int TwoWire::i2c_master_write(int address, const char *data, int length, int stop)
{

    return 0;
}

int TwoWire::i2c_master_byte_read(int *value, int last)
{

    return 0;
}

int TwoWire::i2c_master_byte_write(int data)
{

    return 1;
}

int TwoWire::i2c_slave_write(const char *data, int length)
{
	int size;

    return size;
}

void TwoWire::handleInterrupt(uint8_t TransferDirection, uint16_t AddrMatchCode)
{
}

void TwoWire::enableInterrupt(void)
{
}

void TwoWire::disableInterrupt(void)
{
}


// Preinstantiate Objects //////////////////////////////////////////////////////

#if BOARD_NR_I2C > 0
TwoWire Wire  = TwoWire(HAL_I2C1);
#endif
#if BOARD_NR_I2C > 1
TwoWire Wire1 = TwoWire(HAL_I2C2);
#endif
#if BOARD_NR_I2C > 2
TwoWire Wire2 = TwoWire(HAL_I2C3);
#endif
