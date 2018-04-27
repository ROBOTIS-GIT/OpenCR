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


// Constructors ////////////////////////////////////////////////////////////////
UARTClass::UARTClass(void){

}

UARTClass::UARTClass(uint8_t uart_num, uint8_t uart_mode)
{
  _uart_num  = uart_num;
  _uart_mode = uart_mode;
  _uart_baudrate = 0;
  rx_cnt = 0;
  tx_cnt = 0;
}

void UARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, Mode_8N1);
}

void UARTClass::begin(const uint32_t dwBaudRate, const UARTModes config)
{
  UNUSED(config);

  rx_buffer.iHead = rx_buffer.iTail = 0;
  tx_buffer.iHead = tx_buffer.iTail = 0;

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
  while (tx_buffer.iHead != tx_buffer.iTail); //wait for transmit data to be sent
  // Wait for transmission to complete
}

size_t UARTClass::write( const uint8_t uc_data )
{
  tx_cnt++;
  return drv_uart_write(_uart_num, uc_data);
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
  /*
  if( _uart_mode == DRV_UART_IRQ_MODE )
  {
    if (tx_buffer.iHead != tx_buffer.iTail)
    {
  		unsigned char c = tx_buffer.buffer[tx_buffer.iTail];
  		tx_buffer.iTail = (uint16_t)(tx_buffer.iTail + 1) % SERIAL_BUFFER_SIZE;
  		HAL_UART_Transmit_IT(_pUart, (uint8_t *)&c, 1);
    }
  }
  */
}
