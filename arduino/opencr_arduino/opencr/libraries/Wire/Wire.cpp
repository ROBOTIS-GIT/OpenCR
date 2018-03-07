/*
   TwoWire.cpp - A Wire-like wrapper for SlowSoftI2CMaster.

   It is derived from the Arduino Wire library and for this reason is
   published under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation; either version 2.1 of
   the License, or (at your option) any later version.
*/

#include <Wire.h>


TwoWire::TwoWire(uint8_t sda, uint8_t scl) {
  si2c = new SlowSoftI2CMaster(sda, scl);
}

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  error = 0;
  transmitting = false;

  si2c->i2c_init();
}

void TwoWire::begin(uint8_t sda_pin, uint8_t scl_pin) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  error = 0;
  transmitting = false;

  si2c->i2c_init(sda_pin, scl_pin);
}

void  TwoWire::end(void) {
  free(si2c);
}

void  TwoWire::setClock(uint32_t _) {
}

void  TwoWire::beginTransmission(uint8_t address) {
  if (transmitting) {
    error = (si2c->i2c_rep_start((address<<1)|I2C_WRITE) ? 0 : 2);
  } else {
    error = (si2c->i2c_start((address<<1)|I2C_WRITE) ? 0 : 2);
  }
  // indicate that we are transmitting
  transmitting = 1;
}

void  TwoWire::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

uint8_t  TwoWire::endTransmission(uint8_t sendStop)
{
  uint8_t transError = error;
  if (sendStop) {
    si2c->i2c_stop();
    transmitting = 0;
  }
  error = 0;
  return transError;
}

//  This provides backwards compatibility with the original
//  definition, and expected behaviour, of endTransmission
//
uint8_t  TwoWire::endTransmission(void)
{
  return endTransmission(true);
}

size_t  TwoWire::write(uint8_t data) {
  if (si2c->i2c_write(data)) {
    return 1;
  } else {
    if (error == 0) error = 3;
    return 0;
  }
}

size_t  TwoWire::write(const uint8_t *data, size_t quantity) {
  size_t trans = 0;
  for(size_t i = 0; i < quantity; ++i){
    trans += write(data[i]);
  }
  return trans;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
        uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  uint8_t localerror = 0;
  if (isize > 0) {
    // send internal address; this mode allows sending a repeated start to access
    // some devices' internal registers. This function is executed by the hardware
    // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)
    beginTransmission(address);
    // the maximum size of internal address is 3 bytes
    if (isize > 3){
      isize = 3;
    }
    // write internal register address - most significant byte first
    while (isize-- > 0)
      write((uint8_t)(iaddress >> (isize*8)));
    endTransmission(false);
  }
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  localerror = !si2c->i2c_rep_start((address<<1) | I2C_READ);
  if (error == 0 && localerror) error = 2;
  // perform blocking read into buffer
  for (uint8_t cnt=0; cnt < quantity; cnt++)
    rxBuffer[cnt] = si2c->i2c_read(cnt == quantity-1);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = quantity;
  if (sendStop) {
    transmitting = 0;
    si2c->i2c_stop();
  }
  return quantity;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}


uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  return value;
}

int TwoWire::peek(void) {
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void TwoWire::flush(void) {
}



TwoWire Wire(14, 15);
