/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    M Sproul    10/08/29 Changed #ifdefs from cpu to register
0008    S Kanemoto  12/06/22 Fixed for Leonardo by @maris_HY
0009    J Reucker   15/04/10 Issue #292 Fixed problems with ATmega8 (thanks to Pete62)
0010    jipp        15/04/13 added additional define check #2923
0011    Baram       16/10/06 fixed for OpenCR
*************************************************/

#include  <chip.h>
#include "variant.h"
#include "Tone.h"


// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)

volatile int32_t  tone_toggle_count;
volatile uint8_t  tone_timer = TIMER_TONE;
volatile uint8_t  tone_pin;
volatile uint8_t  tone_pin_out;
volatile bool     tone_enable = false;


void tone_isr( void );



static void toneBegin(uint8_t _pin)
{
  tone_pin = _pin;
  tone_pin_out = 0;
  tone_enable = true;
}


static void toneEnd(void)
{
  tone_enable = false;
  tone_pin_out = 0;
  drv_timer_pause(tone_timer);
}


// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  //uint8_t prescalarbits = 0b001;
  long toggle_count = 0;
  uint32_t ocr = 0;
  //int8_t _timer;


  // Set the pinMode as OUTPUT
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, 0);

  ocr = (1000000 / frequency) / 2 ;

  // Calculate the toggle count
  if (duration > 0)
  {
    toggle_count = 2 * frequency * duration / 1000;
  }
  else
  {
    toggle_count = -1;
  }

  tone_toggle_count = toggle_count;

  toneBegin(_pin);

  drv_timer_set_period(tone_timer, ocr);
  drv_timer_attachInterrupt(tone_timer, tone_isr);
  drv_timer_resume(tone_timer);
}


void noTone(uint8_t _pin)
{
  toneEnd();
  digitalWrite(_pin, 0);
}


void tone_isr( void )
{
  if (tone_toggle_count != 0)
  {
    // toggle the pin
    tone_pin_out ^= 1;
    digitalWrite(tone_pin, tone_pin_out);

    if(tone_toggle_count > 0)
    {
      tone_toggle_count--;
    }
  }
  else
  {
    toneEnd();
  }
}
