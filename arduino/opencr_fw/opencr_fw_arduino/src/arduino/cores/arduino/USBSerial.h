/****************************************************************************
 *
 * USBSerial core library for Arduino STM32 + HAL + CubeMX (HALMX).
 *
 * Copyright (c) 2016 by Vassilis Serasidis <info@serasidis.gr>
 * Home: http://www.serasidis.gr
 * email: avrsite@yahoo.gr
 *
 * Arduino_STM32 forum: http://www.stm32duino.com
 *
 * The USBSerial.h file follows the function prototypes of
 * the Arduino CDC.h file that was written by Peter Barrett
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
 * BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
 * ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
 *
 ****************************************************************************/
 /*
 *  Modified on: 2016. 7.12.
 *       Author: Baram, PBHP
 */

#ifndef _SERIAL_USB_H_INCLUDED
#define _SERIAL_USB_H_INCLUDED

#include  <chip.h>

#include "RingBuffer.h"
#include "Stream.h"
#include "chip.h"

class USBSerial : public Stream {

  public:
    USBSerial();
    void begin(uint32_t baud_count);
    void begin(uint32_t baud_count, uint8_t config);
    void end(void);

    virtual int available(void);
    //virtual void accept(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t c);
    virtual size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) from Print
    operator bool();

    uint32_t getBaudRate(void);
    uint32_t getRxCnt(void);
    uint32_t getTxCnt(void);
    uint32_t getRxErrCnt(void);
    uint32_t getTxErrCnt(void);

  private:
    uint32_t baudrate;
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t rx_err_cnt;
    uint32_t tx_err_cnt;

};

#endif
