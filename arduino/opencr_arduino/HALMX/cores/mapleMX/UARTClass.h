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

#ifndef _UART_CLASS_
#define _UART_CLASS_

#include "HardwareSerial.h"

// Includes [S]Atmel[/S] STM CMSIS
#include <chip.h>
/* #include "stm32f4xx_hal.h"  breaks the upper level abstraction model */

//#include "variant.h"


#define SERIAL_8N1 UARTClass::Mode_8N1
#define SERIAL_8E1 UARTClass::Mode_8E1
#define SERIAL_8O1 UARTClass::Mode_8O1
#define SERIAL_8M1 UARTClass::Mode_8M1
#define SERIAL_8S1 UARTClass::Mode_8S1

#define SERIAL_BUFFER_SIZE 128

class UARTClass : public HardwareSerial
{
  public:
    enum UARTModes {
      Mode_8N1 = 0, //US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO,
      Mode_8E1, // = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_EVEN,
      Mode_8O1, // = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_ODD,
      Mode_8M1, // = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_MARK,
      Mode_8S1 // = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_SPACE,
    };
    UARTClass(UART_HandleTypeDef *pUart, IRQn_Type dwIrq, uint32_t dwId);
    UARTClass(UART_HandleTypeDef *pUart, IRQn_Type dwIrq, uint32_t dwId, USART_TypeDef* usartNumber );
    UARTClass(void);
    void begin(const uint32_t dwBaudRate);
    void begin(const uint32_t dwBaudRate, const UARTModes config);
    void end(void);
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(const uint8_t c);
    using Print::write; // pull in write(str) and write(buf, size) from Print

    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

    void RxHandler(void); /* Vassilis Serasidis */
    void TxHandler(void); /* Vassilis Serasidis */

    operator bool() { return true; }; // UART always active

  protected:
    void init(const uint32_t dwBaudRate, const uint32_t config);
    
    struct ring_buffer
    {
      uint8_t buffer[SERIAL_BUFFER_SIZE];
      volatile uint16_t iHead;
      volatile uint16_t iTail;
    };
    
    UART_HandleTypeDef *_pUart;
    IRQn_Type _dwIrq;
    uint32_t _dwId;
    USART_TypeDef* _usartNumber;
    uint8_t r_byte;
    ring_buffer tx_buffer = { { 0 }, 0, 0};
    ring_buffer rx_buffer = { { 0 }, 0, 0};

};

#endif // _UART_CLASS_
