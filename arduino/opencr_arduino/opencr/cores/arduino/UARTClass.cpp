/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "UARTClass.h"
#include "wiring_digital.h"
#include "wiring_constants.h"
#include "variant.h"
#include "digitalWriteFast.h"
// Constructors ////////////////////////////////////////////////////////////////
UARTClass::UARTClass(void){

}

UARTClass::UARTClass(uint8_t uart_num, uint8_t uart_mode, uint8_t *txBuffer, uint16_t tx_buffer_size)
{
  _uart_num  = uart_num;
  _uart_mode = uart_mode;
  _uart_baudrate = 0;
  rx_cnt = 0;
  tx_write_size = 0;
  _transmit_pin_BSRR = 0;  // Assume no transmit pin selected...
  tx_buffer.buffer = txBuffer;
  tx_buffer.buffer_size = tx_buffer_size;
}

void UARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, Mode_8N1);
}

void UARTClass::begin(const uint32_t dwBaudRate, const UARTModes config)
{
  UNUSED(config);

  rx_buffer.iHead = rx_buffer.iTail = 0;
  tx_buffer.iHead = 0;
  tx_buffer.iTail = 0;

  _uart_baudrate = dwBaudRate;

  drv_uart_begin(_uart_num, _uart_mode, dwBaudRate);
}

void UARTClass::end( void )
{
  // Clear any received data
  rx_buffer.iHead = rx_buffer.iTail;

  // Wait for any outstanding data to be sent
  flush();
}

int UARTClass::available( void )
{
  if(drv_uart_get_mode(_uart_num) == DRV_UART_IRQ_MODE )
  {
    return (uint32_t)(SERIAL_BUFFER_SIZE + rx_buffer.iHead - rx_buffer.iTail) % SERIAL_BUFFER_SIZE;
  }
  else
  {
    return drv_uart_available(_uart_num);
  }
}

int UARTClass::availableForWrite(void)
{
  int head = tx_buffer.iHead;
  int tail = tx_buffer.iTail;
  if (head >= tail) return SERIAL_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

int UARTClass::peek( void )
{
  if ( rx_buffer.iHead == rx_buffer.iTail )
    return -1;

  return rx_buffer.buffer[rx_buffer.iTail];
}

int UARTClass::read( void )
{
  if(drv_uart_get_mode(_uart_num) == DRV_UART_IRQ_MODE )
  {
    // if the head isn't ahead of the tail, we don't have any characters
    if ( rx_buffer.iHead == rx_buffer.iTail )
      return -1;

    uint8_t uc = rx_buffer.buffer[rx_buffer.iTail];
    rx_buffer.iTail = (unsigned int)(rx_buffer.iTail + 1) % SERIAL_BUFFER_SIZE;
    rx_cnt++;
    return uc;
  }
  else
  {
    rx_cnt++;
    return drv_uart_read(_uart_num);
  }
}

void UARTClass::flush( void )
{
  while (tx_write_size); //wait for transmit data to be sent
}

size_t UARTClass::write( const uint8_t uc_data )
{
  return write(&uc_data, 1); // Lets call the buffer function to do the main work
}
void inline UARTClass::startNextTransmitDMAorIT()
{
    tx_write_size = (tx_buffer.iTail < tx_buffer.iHead)? tx_buffer.iHead-tx_buffer.iTail : tx_buffer.buffer_size - tx_buffer.iTail;
    if ( _uart_mode == DRV_UART_IRQ_MODE) {
      if (drv_uart_write_it(_uart_num, &tx_buffer.buffer[tx_buffer.iTail], tx_write_size) != HAL_OK) 
      {
        tx_write_size = 0;  // error so clear it out
      }
    }
    else
    {
      if (drv_uart_write_dma(_uart_num, &tx_buffer.buffer[tx_buffer.iTail], tx_write_size) != HAL_OK)
      {
        Serial.println("Serial DMA write fail");
        tx_write_size = 0;  // error so clear it out
      }
    }
}

size_t UARTClass::write( const uint8_t *buffer, size_t size )
{
  tx_cnt += size;
  size_t cbLeft = size; 

  // Lets try to put as much of this data into our TX buffer as possible. 
  while (cbLeft--) 
  {
    uint16_t nextWrite = (tx_buffer.iHead + 1) % tx_buffer.buffer_size;
    if (tx_buffer.iTail == nextWrite) 
    {
      // See if we have an active TX or not
      if (!tx_write_size)
      {
        if (_transmit_pin_BSRR)
        {
//          digitalWriteFast(2, HIGH);
          *(_transmit_pin_BSRR) =  _transmit_pin_abstraction;  // Set Transmit pin HIGH after we complete
        }
        startNextTransmitDMAorIT();
      }
      // right now this will wait for the entire previous write to complete before continue...
      while (tx_buffer.iTail == nextWrite) 
      {

      }
    }
    tx_buffer.buffer[tx_buffer.iHead] = *buffer++;
    tx_buffer.iHead = nextWrite;

  }
  // we finished putting stuff on queue, so see if we need to start up write
  // or if it is already going. 
  if (!tx_write_size)
  {
    if (_transmit_pin_BSRR)
    {
//    digitalWriteFast(2, HIGH);
    *(_transmit_pin_BSRR) =  _transmit_pin_abstraction;  // Set Transmit pin HIGH after we complete
    }
    startNextTransmitDMAorIT();
  }
  return size; 
}


uint32_t UARTClass::getBaudRate( void )
{
  return _uart_baudrate;
}

uint32_t UARTClass::getRxCnt(void)
{
  return rx_cnt;
}

uint32_t UARTClass::getTxCnt(void)
{
  return tx_cnt;
}

void UARTClass::transmitterEnable(uint8_t pin) 
{
  // Validate Pin?  Should define max pin in variant?  
  if (pin < 100) 
  {
    pinMode(pin, OUTPUT); // make sure enabled as output pin. 
    _transmit_pin_BSRR = &g_Pin2PortMapArray[pin].GPIOx_Port->BSRR;
    _transmit_pin_abstraction = g_Pin2PortMapArray[pin].Pin_abstraction;
  }
  else 
  {
    _transmit_pin_BSRR = NULL;  // no output pin...
  }
}

void UARTClass::RxHandler (void)
{


  if( _uart_mode == DRV_UART_IRQ_MODE )
  {

    if(available() < (SERIAL_BUFFER_SIZE - 1))
    {
      drv_uart_read_buf(_uart_num, &r_byte, 1);
      rx_buffer.buffer[rx_buffer.iHead] = r_byte;
  		rx_buffer.iHead = (uint16_t)(rx_buffer.iHead + 1) % SERIAL_BUFFER_SIZE;
    }
    drv_uart_start_rx(_uart_num);
  }
}

void UARTClass::TxHandler(void)
{
  // We completed previous write, so update our tail pointer by count
  tx_buffer.iTail += tx_write_size; 
  if (tx_buffer.iTail >= tx_buffer.buffer_size) 
    tx_buffer.iTail = 0;  // Should only wrap to start by our other stuff...

  if (tx_buffer.iHead != tx_buffer.iTail)
  {
    startNextTransmitDMAorIT();
  }
  else 
  {
    // finished all outstanding writes so lets set count to 0
    tx_write_size = 0;

    if (_transmit_pin_BSRR)
    {
      *(_transmit_pin_BSRR) =  (_transmit_pin_abstraction << 16);  // Set Transmit pin low after we complete
    }
//    digitalWriteFast(2, LOW);
  }
}
