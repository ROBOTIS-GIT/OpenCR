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
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * Some functions follow the sam and samd arduino core libray files.
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

#include  <chip.h>

#include <USBSerial.h>
#include "variant.h"


extern uint32_t usb_cdc_bitrate;
extern uint32_t usb_cdc_debug_cnt[];


USBSerial::USBSerial(){
  baudrate = 0;
  rx_cnt = 0;
  tx_cnt = 0;
  rx_err_cnt = 0;
  tx_err_cnt = 0;
}

void USBSerial::begin(uint32_t baud_count){
  UNUSED(baud_count);
}

void USBSerial::begin(uint32_t baud_count, uint8_t config){
  UNUSED(baud_count);
  UNUSED(config);
}

void USBSerial::end(void){
}


int USBSerial::available(void){
  return vcp_is_available();
}

int USBSerial::peek(void)
{
  return vcp_peek();
}

int USBSerial::read(void)
{
  if ( vcp_is_available() == 0 )
    return -1;

  rx_cnt++;

  return vcp_getch();
}

void USBSerial::flush(void){
  while( vcp_is_transmitted() == FALSE );
}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{
  uint32_t length;

  length = vcp_write((uint8_t *)buffer, (uint32_t)size);

  tx_cnt += length;
  
  return (size_t)length;
}


size_t USBSerial::write(uint8_t c) {
	return write(&c, 1);
}

uint32_t USBSerial::getBaudRate(void)
{
  return usb_cdc_bitrate;
}

uint32_t USBSerial::getRxCnt(void)
{
  return rx_cnt;
}

uint32_t USBSerial::getTxCnt(void)
{
  return tx_cnt;
}

uint32_t USBSerial::getRxErrCnt(void)
{
  return usb_cdc_debug_cnt[0];
}

uint32_t USBSerial::getTxErrCnt(void)
{
  return usb_cdc_debug_cnt[1];
}


// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
USBSerial::operator bool()
{
  if( vcp_is_connected() == TRUE ) return true;
  else                             return false;
}
