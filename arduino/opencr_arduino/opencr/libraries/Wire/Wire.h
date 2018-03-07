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

#ifndef TwoWire_h
#define TwoWire_h

#include "variant.h"
#include <inttypes.h>
#include "Stream.h"
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
  void setClock(uint32_t _);
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


extern TwoWire Wire;

#endif
