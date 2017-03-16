/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#include  <chip.h>

#include <USBSerial.h>
#include "drv_timer.h"
#include "variant.h"
#include "HardwareTimer.h"


#define MAX_RELOAD ((1 << 16) - 1)

HardwareTimer::HardwareTimer(uint8_t timerNum) {
  if (timerNum > TIMER_CH_MAX) {
    tim_num = 0;
  }

  tim_num = timerNum;
}

void HardwareTimer::pause(void) {
  drv_timer_pause(tim_num);
}

void HardwareTimer::resume(void) {
  drv_timer_resume(tim_num);
}

void HardwareTimer::stop(void) {
  drv_timer_pause(tim_num);
}

void HardwareTimer::start(void) {
  drv_timer_resume(tim_num);
}

uint16_t HardwareTimer::setPeriod(uint32_t microseconds) {

  drv_timer_set_period(tim_num, microseconds);

  return 0;
}

void HardwareTimer::attachInterrupt(voidFuncPtr handler) {
  drv_timer_attachInterrupt(tim_num, handler);
}

void HardwareTimer::detachInterrupt(void) {
    drv_timer_detachInterrupt(tim_num);
}

void HardwareTimer::refresh(void) {
    drv_timer_refresh(tim_num);
}
