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
  tx_cnt = 0;
  tx_write_size = 0;
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

#ifdef DRV_UART_RX_DMA_ONLY
  rx_buffer.iHead = rx_buffer.iTail = 0;
#endif
  tx_buffer.iHead = 0;
  tx_buffer.iTail = 0;

  _uart_baudrate = dwBaudRate;

  drv_uart_begin(_uart_num, _uart_mode, dwBaudRate);
}

void UARTClass::end( void )
{
  // Clear any received data
#ifdef DRV_UART_RX_DMA_ONLY
  rx_buffer.iHead = rx_buffer.iTail;
#endif
  // Wait for any outstanding data to be sent
  flush();
}

int UARTClass::available( void )
{
#ifdef DRV_UART_RX_DMA_ONLY
  if(drv_uart_get_mode(_uart_num) == DRV_UART_IRQ_MODE )
  {
    return (uint32_t)(SERIAL_BUFFER_SIZE + rx_buffer.iHead - rx_buffer.iTail) % SERIAL_BUFFER_SIZE;
  }
  else
  {
    return drv_uart_available(_uart_num);
  }
#else
  return drv_uart_available(_uart_num);
#endif  
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
#ifdef DRV_UART_RX_DMA_ONLY
  if(drv_uart_get_mode(_uart_num) == DRV_UART_IRQ_MODE )
  {
    if ( rx_buffer.iHead == rx_buffer.iTail )
      return -1;

    return rx_buffer.buffer[rx_buffer.iTail];
  }
  else 
  {
    return drv_uart_peek(_uart_num);
  }
#else
  return drv_uart_peek(_uart_num);
#endif  
}

int UARTClass::read( void )
{
#ifdef DRV_UART_RX_DMA_ONLY
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
    int return_value = drv_uart_read(_uart_num);
    if (return_value != -1) 
    {
      rx_cnt++;
    }
    return return_value; 
  }
#else
  int return_value = drv_uart_read(_uart_num);
  if (return_value != -1) 
  {
    rx_cnt++;
  }
  return return_value; 
#endif  
}

void UARTClass::flush( void )
{
  while (tx_write_size); //wait for transmit data to be sent
}

void UARTClass::flushRx( uint32_t timeout_ms )
{
#ifdef DRV_UART_RX_DMA_ONLY
  if(drv_uart_get_mode(_uart_num) == DRV_UART_IRQ_MODE )
  {
    // sort of hack, wait the time specified and then just clear it
    if (timeout_ms) 
    {
      uint32_t pre_time_ms = millis();
      while((millis() - pre_time_ms) < timeout_ms)
      {
      }

    }
    rx_buffer.iTail = rx_buffer.iHead;  // clear out buffer
  }
  else 
  {
    drv_uart_rx_flush(_uart_num, timeout_ms);
  }
#else
  drv_uart_rx_flush(_uart_num, timeout_ms);
#endif  
}

size_t UARTClass::write( const uint8_t uc_data )
{
  return write(&uc_data, 1); // Lets call the buffer function to do the main work
}
void inline UARTClass::startNextTransmitDMAorIT()
{
  tx_write_size = (tx_buffer.iTail < tx_buffer.iHead)? tx_buffer.iHead-tx_buffer.iTail : tx_buffer.buffer_size - tx_buffer.iTail;
  if (drv_uart_write_dma_it(_uart_num, &tx_buffer.buffer[tx_buffer.iTail], tx_write_size) != 0) 
  {
    tx_write_size = 0;  // error so clear it out
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


void UARTClass::RxHandler (void)
{
#ifdef DRV_UART_RX_DMA_ONLY
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
#endif
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
  }
}
