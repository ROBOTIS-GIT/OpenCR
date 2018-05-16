/*
  digitalWriteFast.h - A faster digitalWrite and digitalRead
  ...based partially on digitalWrite fast code by Paul Stoffregen,
  as part of Teensyduino and partially based off of the earlier
  arduino version http://code.google.com/p/digitalwritefast

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
*/
/* Only want to process this if pin count has been defined which inplies that the full variant.h has been read */

#ifdef PINS_COUNT 
#ifndef _DIGITAL_WRITE_FAST_
#define _DIGITAL_WRITE_FAST_


#ifdef __cplusplus
 extern "C" {
#endif

static inline void digitalWriteFast(uint8_t, uint8_t) __attribute__((always_inline, unused));
static inline void digitalWriteFast(uint8_t pin, uint8_t val)
{
  if (__builtin_constant_p(pin)) {
    if (val) {
      if(pin < PINS_COUNT - 1) {
        g_Pin2PortMapArray[pin].GPIOx_Port->BSRR = g_Pin2PortMapArray[pin].Pin_abstraction;
      }
    } else {
      if (pin < PINS_COUNT - 1) {
        g_Pin2PortMapArray[pin].GPIOx_Port->BSRR = (g_Pin2PortMapArray[pin].Pin_abstraction << 16);
      }
    }
  } else {
    digitalWrite(pin, val);
  }
}


static inline int digitalReadFast(uint8_t) __attribute__((always_inline, unused));
static inline int digitalReadFast(uint8_t pin)
{
  if (__builtin_constant_p(pin)) {
    if (pin < PINS_COUNT - 1) {
      return (g_Pin2PortMapArray[pin].GPIOx_Port->IDR & g_Pin2PortMapArray[pin].Pin_abstraction)? HIGH : LOW;
    }
  } 
  return digitalRead(pin);
}


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus


#endif /* _DIGITAL_WRITE_FAST_ */
#endif /* PINS_COUNT */