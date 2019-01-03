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
//#define USE_SLOW_SOFT_I2C_MASTER

#ifndef TwoWire_h
#define TwoWire_h

#include "variant.h"
#include <inttypes.h>
#include "Stream.h"
#ifdef USE_SLOW_SOFT_I2C_MASTER
#include <SlowSoftI2CMaster.h>


class TwoWire : public Stream
{
private:
  uint8_t rxBuffer[BUFFER_LENGTH];
  uint8_t rxBufferIndex;
  uint8_t rxBufferLength;
  uint8_t transmitting;
  uint8_t error;
  SlowSoftI2CMaster *si2c;
public:
  TwoWire(uint8_t sda, uint8_t scl);
  void begin(void);
  void begin(uint8_t sda_pin, uint8_t scl_pin);
  void end(void);
  void setClock(uint32_t frequency);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  uint8_t endTransmission(uint8_t sendStop);
  uint8_t endTransmission(void);
  size_t write(uint8_t data);
  size_t write(const uint8_t *data, size_t quantity);
  uint8_t requestFrom(uint8_t address, uint8_t quantity,
          uint32_t iaddress, uint8_t isize, uint8_t sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
  uint8_t requestFrom(int address, int quantity, int sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  uint8_t requestFrom(int address, int quantity);
  int available(void);
  int read(void);
  int peek(void);
  void flush(void);
  inline size_t write(unsigned long n) { return write((uint8_t)n); }
  inline size_t write(long n) { return write((uint8_t)n); }
  inline size_t write(unsigned int n) { return write((uint8_t)n); }
  inline size_t write(int n) { return write((uint8_t)n); }
  using Print::write;
};

#else  
//=============================================================================
// Lets try hardware I2C
//=============================================================================
// Some temporary DEBUG defines
//#define WIRE_USE_DEBUG_IO_PINS  // If defined enables debug
#define WIRE_DEBUG_RECEIVE_FROM 11
#define WIRE_DEBUG_END_TRANSFER 9
// Some default buffer length and timeouts 
#define BUFFER_LENGTH 32
#define WIRE_TX_TIMEOUT 1000 // timeout in ms
#define WIRE_RX_TIMEOUT 1000 // timeout in ms



//=============================================================================
// Define some external call backs and interrupt handlers
//=============================================================================
extern void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
extern void HAL_I2C_SlaveAddrCpltCallback(I2C_HandleTypeDef *hi2c);
extern "C" void I2C1_EV_IRQHandler(void);
extern "C" void I2C2_EV_IRQHandler(void);
extern "C" void I2C1_ER_IRQHandler(void);
extern "C" void I2C2_ER_IRQHandler(void);

//=============================================================================
// TwoWire class definition
//=============================================================================
class TwoWire : public Stream
{

public:
  friend void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
  friend void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
  friend void HAL_I2C_SlaveAddrCpltCallback(I2C_HandleTypeDef *hi2c);
  friend void I2C1_EV_IRQHandler(void);
  friend void I2C2_EV_IRQHandler(void);
  friend void I2C1_ER_IRQHandler(void);
  friend void I2C2_ER_IRQHandler(void);

  TwoWire(I2C_HandleTypeDef *hi2c);
  void begin(void);
  void begin(uint8_t address);
  void begin(int address) {
    begin((uint8_t)address);
  }
  void end(void);
  void setClock(uint32_t frequency);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  uint8_t endTransmission(uint8_t sendStop);
  uint8_t endTransmission(void);
  size_t write(uint8_t data);
  size_t write(const uint8_t *data, size_t quantity);
//  uint8_t requestFrom(uint8_t address, uint8_t quantity,
//          uint32_t iaddress, uint8_t isize, uint8_t sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
  uint8_t requestFrom(int address, int quantity, int sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  uint8_t requestFrom(int address, int quantity);
  int available(void);
  int read(void);
  int peek(void);
  void flush(void);
  inline size_t write(unsigned long n) { return write((uint8_t)n); }
  inline size_t write(long n) { return write((uint8_t)n); }
  inline size_t write(unsigned int n) { return write((uint8_t)n); }
  inline size_t write(int n) { return write((uint8_t)n); }
  using Print::write;

  void onReceive(void (*function)(int numBytes));
  void onRequest(void (*function)(void));
  // Process Client ISR Callbacks    
  void processTXCallback(void);
  void processRXCallback(void);  
  void processAddrCallback(void);


private:
  // Helper functions brought over from HAL I2C code
  HAL_StatusTypeDef waitOnTXISFlagUntilTimeout(uint32_t Timeout);
  HAL_StatusTypeDef waitOnRXNEFlagUntilTimeout(uint32_t Timeout);
  HAL_StatusTypeDef waitOnFlagUntilTimeout(uint32_t flag, uint32_t Timeout);
  HAL_StatusTypeDef isAcknowledgeFailed(uint32_t Timeout);

  void transferConfig(uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

  uint8_t rxBuffer[BUFFER_LENGTH];
  uint8_t rxBufferIndex;
  uint8_t rxBufferLength;

  uint16_t device_address;
  uint32_t frequency_;
  uint8_t txBuffer[BUFFER_LENGTH];
  uint8_t txBufferIndex;
  uint8_t txBufferLength;
  uint8_t transmitting;
  uint8_t error;
  uint8_t slave_mode;
  uint8_t state_;                      // What state are we in 0=not active 1=begin() called
  uint8_t sendStop_;                  // Last communications do sendStop?
  I2C_HandleTypeDef *hi2c_;           // Handle to the hardware I2c...
  void (*user_onRequest)(void);
  void (*user_onReceive)(int);

};

#endif
//=============================================================================
// Define standard objects
//=============================================================================
extern TwoWire Wire;
extern TwoWire Wire1;

#endif
