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


USBSerial::USBSerial(){
}

void USBSerial::begin(uint32_t baud_count){
}

void USBSerial::begin(uint32_t baud_count, uint8_t config){
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

  return vcp_getch();
}

void USBSerial::flush(void){

}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{
  return (size_t)vcp_write((uint8_t *)buffer, (uint32_t)size);
}


size_t USBSerial::write(uint8_t c) {
	return write(&c, 1);
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

