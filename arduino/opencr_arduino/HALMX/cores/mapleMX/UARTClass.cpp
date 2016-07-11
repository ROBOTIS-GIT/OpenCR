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

UARTClass::UARTClass( UART_HandleTypeDef *pUart, IRQn_Type dwIrq, uint32_t dwId )
{
  _usartNumber = USART1; //In this case create by default the USART1 serial port.
  _pUart=pUart;
  _dwIrq=dwIrq;
  _dwId=dwId;
}

 /**
  * Additional constructor by Vassilis Serasidis
  * 
  */
UARTClass::UARTClass( UART_HandleTypeDef *pUart, IRQn_Type dwIrq, uint32_t dwId, USART_TypeDef* usartNumber )
{
  _pUart=pUart;
  _dwIrq=dwIrq;
  _dwId=dwId;
  _usartNumber = usartNumber;
}
// Public Methods //////////////////////////////////////////////////////////////

void UARTClass::begin(const uint32_t dwBaudRate)
{
  begin(dwBaudRate, Mode_8N1);
}

void UARTClass::begin(const uint32_t dwBaudRate, const UARTModes config)
{
  uint32_t modeReg = 0;//static_cast<uint32_t>(config) & 0x00000E00;
  init(dwBaudRate, modeReg);  
} 

void UARTClass::init(const uint32_t dwBaudRate, const uint32_t modeReg)
{
  /** Configure baudrate (asynchronous, no oversampling)
   *  02 March 2016 by Vassilis Serasidis
   */
  _pUart->Instance = _usartNumber;
  _pUart->Init.BaudRate = dwBaudRate;
  _pUart->Init.WordLength = UART_WORDLENGTH_8B;
  _pUart->Init.StopBits = UART_STOPBITS_1;
  _pUart->Init.Parity = UART_PARITY_NONE;
  _pUart->Init.Mode = UART_MODE_TX_RX;
  _pUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  _pUart->Init.OverSampling = UART_OVERSAMPLING_16;


  // Configure interrupts
  // Enable UART interrupt in NVIC  when we have enough info for bridge
  NVIC_EnableIRQ(_dwIrq);

  // Make sure both ring buffers are initialized back to empty.
  rx_buffer.iHead = rx_buffer.iTail = 0;
  tx_buffer.iHead = tx_buffer.iTail = 0;

  /** Enable receiver and transmitter
   *  02 March 2016 by Vassilis Serasidis
   */
  HAL_UART_Init(_pUart);
  
  HAL_UART_Receive_IT(_pUart, (uint8_t *)&r_byte, 1);
}

void UARTClass::end( void )
{
  // Clear any received data
  rx_buffer.iHead = rx_buffer.iTail;

  // Wait for any outstanding data to be sent
  flush();

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( _dwIrq );

}

/* void UARTClass::setInterruptPriority(uint32_t priority)
{
//  NVIC_SetPriority(_dwIrq, priority & 0x0F);
} */

uint32_t UARTClass::getInterruptPriority()
{
  return NVIC_GetPriority(_dwIrq);
}

int UARTClass::available( void )
{
  return (uint32_t)(SERIAL_BUFFER_SIZE + rx_buffer.iHead - rx_buffer.iTail) % SERIAL_BUFFER_SIZE;
}

int UARTClass::availableForWrite(void)
{
/*   int head = tx_buffer.iHead;
  int tail = tx_buffer.iTail;
  if (head >= tail) return SERIAL_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1; */
}

int UARTClass::peek( void )
{
  if ( rx_buffer.iHead == rx_buffer.iTail )
    return -1;

  return rx_buffer.buffer[rx_buffer.iTail];
}

int UARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( rx_buffer.iHead == rx_buffer.iTail )
    return -1;

  uint8_t uc = rx_buffer.buffer[rx_buffer.iTail];
  rx_buffer.iTail = (unsigned int)(rx_buffer.iTail + 1) % SERIAL_BUFFER_SIZE;
  return uc;
}

void UARTClass::flush( void )
{
  while (tx_buffer.iHead != tx_buffer.iTail); //wait for transmit data to be sent
  // Wait for transmission to complete

}

size_t UARTClass::write( const uint8_t uc_data )
{
  if(HAL_UART_Transmit_IT(_pUart, (uint8_t *)&uc_data, 1) == HAL_BUSY){
    tx_buffer.buffer[tx_buffer.iHead] = uc_data;
    tx_buffer.iHead = (uint32_t)(tx_buffer.iHead + 1) % SERIAL_BUFFER_SIZE;
  }
  return 1;
}

/************************************************
 * 02 March 2016 by Vassilis Serasidis
 */
void UARTClass::RxHandler (void){
  
    if(available() < (SERIAL_BUFFER_SIZE - 1)){ //If there is empty space in rx_buffer, read a byte from the Serial port and save it to the buffer.  
    rx_buffer.buffer[rx_buffer.iHead] = r_byte; 
		rx_buffer.iHead = (uint16_t)(rx_buffer.iHead + 1) % SERIAL_BUFFER_SIZE;
  }
  HAL_UART_Receive_IT(_pUart, (uint8_t *)&r_byte, 1); //Get prepared for the next incoming byte.
}

/************************************************
 * 09 May 2016 by Vassilis Serasidis
 */
void UARTClass::TxHandler(void){
  
  if (tx_buffer.iHead != tx_buffer.iTail)	{
		unsigned char c = tx_buffer.buffer[tx_buffer.iTail];
		tx_buffer.iTail = (uint16_t)(tx_buffer.iTail + 1) % SERIAL_BUFFER_SIZE;
		HAL_UART_Transmit_IT(_pUart, (uint8_t *)&c, 1);
  }
}
