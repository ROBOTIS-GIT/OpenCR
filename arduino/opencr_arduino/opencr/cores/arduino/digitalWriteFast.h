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
#ifndef _DIGITAL_WRITE_FAST_
#define _DIGITAL_WRITE_FAST_

#include "wiring_digital.h"

static inline void digitalWriteFast(uint8_t, uint8_t) __attribute__((always_inline, unused));
static inline void digitalWriteFast(uint8_t pin, uint8_t val)
{
  if (__builtin_constant_p(pin)) {
    if (val) {
      if (pin == 0) {
        g_Pin2PortMapArray[0].GPIOx_Port->BSRR = g_Pin2PortMapArray[0].Pin_abstraction;
      } else if (pin == 1) {
        g_Pin2PortMapArray[1].GPIOx_Port->BSRR = g_Pin2PortMapArray[1].Pin_abstraction;
      } else if (pin == 2) {
        g_Pin2PortMapArray[2].GPIOx_Port->BSRR = g_Pin2PortMapArray[2].Pin_abstraction;
      } else if (pin == 3) {
        g_Pin2PortMapArray[3].GPIOx_Port->BSRR = g_Pin2PortMapArray[3].Pin_abstraction;
      } else if (pin == 4) {
        g_Pin2PortMapArray[4].GPIOx_Port->BSRR = g_Pin2PortMapArray[4].Pin_abstraction;
      } else if (pin == 5) {
        g_Pin2PortMapArray[5].GPIOx_Port->BSRR = g_Pin2PortMapArray[5].Pin_abstraction;
      } else if (pin == 6) {
        g_Pin2PortMapArray[6].GPIOx_Port->BSRR = g_Pin2PortMapArray[6].Pin_abstraction;
      } else if (pin == 7) {
        g_Pin2PortMapArray[7].GPIOx_Port->BSRR = g_Pin2PortMapArray[7].Pin_abstraction;
      } else if (pin == 8) {
        g_Pin2PortMapArray[8].GPIOx_Port->BSRR = g_Pin2PortMapArray[8].Pin_abstraction;
      } else if (pin == 9) {
        g_Pin2PortMapArray[9].GPIOx_Port->BSRR = g_Pin2PortMapArray[9].Pin_abstraction;
      } else if (pin == 10) {
        g_Pin2PortMapArray[10].GPIOx_Port->BSRR = g_Pin2PortMapArray[10].Pin_abstraction;
      } else if (pin == 11) {
        g_Pin2PortMapArray[11].GPIOx_Port->BSRR = g_Pin2PortMapArray[11].Pin_abstraction;
      } else if (pin == 12) {
        g_Pin2PortMapArray[12].GPIOx_Port->BSRR = g_Pin2PortMapArray[12].Pin_abstraction;
      } else if (pin == 13) {
        g_Pin2PortMapArray[13].GPIOx_Port->BSRR = g_Pin2PortMapArray[13].Pin_abstraction;
      } else if (pin == 14) {
        g_Pin2PortMapArray[14].GPIOx_Port->BSRR = g_Pin2PortMapArray[14].Pin_abstraction;
      } else if (pin == 15) {
        g_Pin2PortMapArray[15].GPIOx_Port->BSRR = g_Pin2PortMapArray[15].Pin_abstraction;
      } else if (pin == 16) {
        g_Pin2PortMapArray[16].GPIOx_Port->BSRR = g_Pin2PortMapArray[16].Pin_abstraction;
      } else if (pin == 17) {
        g_Pin2PortMapArray[17].GPIOx_Port->BSRR = g_Pin2PortMapArray[17].Pin_abstraction;
      } else if (pin == 18) {
        g_Pin2PortMapArray[18].GPIOx_Port->BSRR = g_Pin2PortMapArray[18].Pin_abstraction;
      } else if (pin == 19) {
        g_Pin2PortMapArray[19].GPIOx_Port->BSRR = g_Pin2PortMapArray[19].Pin_abstraction;
      } else if (pin == 20) {
        g_Pin2PortMapArray[20].GPIOx_Port->BSRR = g_Pin2PortMapArray[20].Pin_abstraction;
      } else if (pin == 21) {
        g_Pin2PortMapArray[21].GPIOx_Port->BSRR = g_Pin2PortMapArray[21].Pin_abstraction;
      } else if (pin == 22) {
        g_Pin2PortMapArray[22].GPIOx_Port->BSRR = g_Pin2PortMapArray[22].Pin_abstraction;
      } else if (pin == 23) {
        g_Pin2PortMapArray[23].GPIOx_Port->BSRR = g_Pin2PortMapArray[23].Pin_abstraction;
      } else if (pin == 24) {
        g_Pin2PortMapArray[24].GPIOx_Port->BSRR = g_Pin2PortMapArray[24].Pin_abstraction;
      } else if (pin == 25) {
        g_Pin2PortMapArray[25].GPIOx_Port->BSRR = g_Pin2PortMapArray[25].Pin_abstraction;
      } else if (pin == 26) {
        g_Pin2PortMapArray[26].GPIOx_Port->BSRR = g_Pin2PortMapArray[26].Pin_abstraction;
      } else if (pin == 27) {
        g_Pin2PortMapArray[27].GPIOx_Port->BSRR = g_Pin2PortMapArray[27].Pin_abstraction;
      } else if (pin == 28) {
        g_Pin2PortMapArray[28].GPIOx_Port->BSRR = g_Pin2PortMapArray[28].Pin_abstraction;
      } else if (pin == 29) {
        g_Pin2PortMapArray[29].GPIOx_Port->BSRR = g_Pin2PortMapArray[29].Pin_abstraction;
      } else if (pin == 30) {
        g_Pin2PortMapArray[30].GPIOx_Port->BSRR = g_Pin2PortMapArray[30].Pin_abstraction;
      } else if (pin == 31) {
        g_Pin2PortMapArray[31].GPIOx_Port->BSRR = g_Pin2PortMapArray[31].Pin_abstraction;
      } else if (pin == 32) {
        g_Pin2PortMapArray[32].GPIOx_Port->BSRR = g_Pin2PortMapArray[32].Pin_abstraction;
      } else if (pin == 33) {
        g_Pin2PortMapArray[33].GPIOx_Port->BSRR = g_Pin2PortMapArray[33].Pin_abstraction;
      } else if (pin == 34) {
        g_Pin2PortMapArray[34].GPIOx_Port->BSRR = g_Pin2PortMapArray[34].Pin_abstraction;
      } else if (pin == 35) {
        g_Pin2PortMapArray[35].GPIOx_Port->BSRR = g_Pin2PortMapArray[35].Pin_abstraction;
      } else if (pin == 36) {
        g_Pin2PortMapArray[36].GPIOx_Port->BSRR = g_Pin2PortMapArray[36].Pin_abstraction;
      } else if (pin == 37) {
        g_Pin2PortMapArray[37].GPIOx_Port->BSRR = g_Pin2PortMapArray[37].Pin_abstraction;
      } else if (pin == 38) {
        g_Pin2PortMapArray[38].GPIOx_Port->BSRR = g_Pin2PortMapArray[38].Pin_abstraction;
      } else if (pin == 39) {
        g_Pin2PortMapArray[39].GPIOx_Port->BSRR = g_Pin2PortMapArray[39].Pin_abstraction;
      } else if (pin == 30) {
        g_Pin2PortMapArray[40].GPIOx_Port->BSRR = g_Pin2PortMapArray[40].Pin_abstraction;
      } else if (pin == 41) {
        g_Pin2PortMapArray[41].GPIOx_Port->BSRR = g_Pin2PortMapArray[41].Pin_abstraction;
      } else if (pin == 42) {
        g_Pin2PortMapArray[42].GPIOx_Port->BSRR = g_Pin2PortMapArray[42].Pin_abstraction;
      } else if (pin == 43) {
        g_Pin2PortMapArray[43].GPIOx_Port->BSRR = g_Pin2PortMapArray[43].Pin_abstraction;
      } else if (pin == 44) {
        g_Pin2PortMapArray[44].GPIOx_Port->BSRR = g_Pin2PortMapArray[44].Pin_abstraction;
      } else if (pin == 45) {
        g_Pin2PortMapArray[45].GPIOx_Port->BSRR = g_Pin2PortMapArray[45].Pin_abstraction;
      } else if (pin == 46) {
        g_Pin2PortMapArray[46].GPIOx_Port->BSRR = g_Pin2PortMapArray[46].Pin_abstraction;
      } else if (pin == 47) {
        g_Pin2PortMapArray[47].GPIOx_Port->BSRR = g_Pin2PortMapArray[47].Pin_abstraction;
      } else if (pin == 48) {
        g_Pin2PortMapArray[48].GPIOx_Port->BSRR = g_Pin2PortMapArray[48].Pin_abstraction;
      } else if (pin == 49) {
        g_Pin2PortMapArray[49].GPIOx_Port->BSRR = g_Pin2PortMapArray[49].Pin_abstraction;
      } else if (pin == 50) {
        g_Pin2PortMapArray[50].GPIOx_Port->BSRR = g_Pin2PortMapArray[50].Pin_abstraction;
      } else if (pin == 51) {
        g_Pin2PortMapArray[51].GPIOx_Port->BSRR = g_Pin2PortMapArray[51].Pin_abstraction;
      } else if (pin == 52) {
        g_Pin2PortMapArray[52].GPIOx_Port->BSRR = g_Pin2PortMapArray[52].Pin_abstraction;
      } else if (pin == 53) {
        g_Pin2PortMapArray[53].GPIOx_Port->BSRR = g_Pin2PortMapArray[53].Pin_abstraction;
      } else if (pin == 54) {
        g_Pin2PortMapArray[54].GPIOx_Port->BSRR = g_Pin2PortMapArray[54].Pin_abstraction;
      } else if (pin == 55) {
        g_Pin2PortMapArray[55].GPIOx_Port->BSRR = g_Pin2PortMapArray[55].Pin_abstraction;
      } else if (pin == 56) {
        g_Pin2PortMapArray[56].GPIOx_Port->BSRR = g_Pin2PortMapArray[56].Pin_abstraction;
      } else if (pin == 57) {
        g_Pin2PortMapArray[57].GPIOx_Port->BSRR = g_Pin2PortMapArray[57].Pin_abstraction;
      } else if (pin == 58) {
        g_Pin2PortMapArray[58].GPIOx_Port->BSRR = g_Pin2PortMapArray[58].Pin_abstraction;
      } else if (pin == 59) {
        g_Pin2PortMapArray[59].GPIOx_Port->BSRR = g_Pin2PortMapArray[59].Pin_abstraction;
      } else if (pin == 60) {
        g_Pin2PortMapArray[60].GPIOx_Port->BSRR = g_Pin2PortMapArray[60].Pin_abstraction;
      } else if (pin == 61) {
        g_Pin2PortMapArray[61].GPIOx_Port->BSRR = g_Pin2PortMapArray[61].Pin_abstraction;
      } else if (pin == 62) {
        g_Pin2PortMapArray[62].GPIOx_Port->BSRR = g_Pin2PortMapArray[62].Pin_abstraction;
      } else if (pin == 63) {
        g_Pin2PortMapArray[63].GPIOx_Port->BSRR = g_Pin2PortMapArray[63].Pin_abstraction;
      } else if (pin == 64) {
        g_Pin2PortMapArray[64].GPIOx_Port->BSRR = g_Pin2PortMapArray[64].Pin_abstraction;
      } else if (pin == 65) {
        g_Pin2PortMapArray[65].GPIOx_Port->BSRR = g_Pin2PortMapArray[65].Pin_abstraction;
      } else if (pin == 66) {
        g_Pin2PortMapArray[66].GPIOx_Port->BSRR = g_Pin2PortMapArray[66].Pin_abstraction;
      } else if (pin == 67) {
        g_Pin2PortMapArray[67].GPIOx_Port->BSRR = g_Pin2PortMapArray[67].Pin_abstraction;
      } else if (pin == 68) {
        g_Pin2PortMapArray[68].GPIOx_Port->BSRR = g_Pin2PortMapArray[68].Pin_abstraction;
      } else if (pin == 69) {
        g_Pin2PortMapArray[69].GPIOx_Port->BSRR = g_Pin2PortMapArray[69].Pin_abstraction;
      } else if (pin == 70) {
        g_Pin2PortMapArray[70].GPIOx_Port->BSRR = g_Pin2PortMapArray[70].Pin_abstraction;
      } else if (pin == 71) {
        g_Pin2PortMapArray[71].GPIOx_Port->BSRR = g_Pin2PortMapArray[71].Pin_abstraction;
      } else if (pin == 72) {
        g_Pin2PortMapArray[72].GPIOx_Port->BSRR = g_Pin2PortMapArray[72].Pin_abstraction;
      } else if (pin == 73) {
        g_Pin2PortMapArray[73].GPIOx_Port->BSRR = g_Pin2PortMapArray[73].Pin_abstraction;
      } else if (pin == 74) {
        g_Pin2PortMapArray[74].GPIOx_Port->BSRR = g_Pin2PortMapArray[74].Pin_abstraction;
      } else if (pin == 75) {
        g_Pin2PortMapArray[75].GPIOx_Port->BSRR = g_Pin2PortMapArray[75].Pin_abstraction;
      } else if (pin == 76) {
        g_Pin2PortMapArray[76].GPIOx_Port->BSRR = g_Pin2PortMapArray[76].Pin_abstraction;
      } else if (pin == 77) {
        g_Pin2PortMapArray[77].GPIOx_Port->BSRR = g_Pin2PortMapArray[77].Pin_abstraction;
      } else if (pin == 78) {
        g_Pin2PortMapArray[78].GPIOx_Port->BSRR = g_Pin2PortMapArray[78].Pin_abstraction;
      } else if (pin == 79) {
        g_Pin2PortMapArray[79].GPIOx_Port->BSRR = g_Pin2PortMapArray[79].Pin_abstraction;
      } else if (pin == 80) {
        g_Pin2PortMapArray[80].GPIOx_Port->BSRR = g_Pin2PortMapArray[80].Pin_abstraction;
      } else if (pin == 81) {
        g_Pin2PortMapArray[81].GPIOx_Port->BSRR = g_Pin2PortMapArray[81].Pin_abstraction;
      } else if (pin == 82) {
        g_Pin2PortMapArray[82].GPIOx_Port->BSRR = g_Pin2PortMapArray[82].Pin_abstraction;
      } else if (pin == 83) {
        g_Pin2PortMapArray[83].GPIOx_Port->BSRR = g_Pin2PortMapArray[83].Pin_abstraction;
      }
    } else {
      if (pin == 0) {
        g_Pin2PortMapArray[0].GPIOx_Port->BSRR = (g_Pin2PortMapArray[0].Pin_abstraction << 16);
      } else if (pin == 1) {
        g_Pin2PortMapArray[1].GPIOx_Port->BSRR = (g_Pin2PortMapArray[1].Pin_abstraction << 16);
      } else if (pin == 2) {
        g_Pin2PortMapArray[2].GPIOx_Port->BSRR = (g_Pin2PortMapArray[2].Pin_abstraction << 16);
      } else if (pin == 3) {
        g_Pin2PortMapArray[3].GPIOx_Port->BSRR = (g_Pin2PortMapArray[3].Pin_abstraction << 16);
      } else if (pin == 4) {
        g_Pin2PortMapArray[4].GPIOx_Port->BSRR = (g_Pin2PortMapArray[4].Pin_abstraction << 16);
      } else if (pin == 5) {
        g_Pin2PortMapArray[5].GPIOx_Port->BSRR = (g_Pin2PortMapArray[5].Pin_abstraction << 16);
      } else if (pin == 6) {
        g_Pin2PortMapArray[6].GPIOx_Port->BSRR = (g_Pin2PortMapArray[6].Pin_abstraction << 16);
      } else if (pin == 7) {
        g_Pin2PortMapArray[7].GPIOx_Port->BSRR = (g_Pin2PortMapArray[7].Pin_abstraction << 16);
      } else if (pin == 8) {
        g_Pin2PortMapArray[8].GPIOx_Port->BSRR = (g_Pin2PortMapArray[8].Pin_abstraction << 16);
      } else if (pin == 9) {
        g_Pin2PortMapArray[9].GPIOx_Port->BSRR = (g_Pin2PortMapArray[9].Pin_abstraction << 16);
      } else if (pin == 10) {
        g_Pin2PortMapArray[10].GPIOx_Port->BSRR = (g_Pin2PortMapArray[10].Pin_abstraction << 16);
      } else if (pin == 11) {
        g_Pin2PortMapArray[11].GPIOx_Port->BSRR = (g_Pin2PortMapArray[11].Pin_abstraction << 16);
      } else if (pin == 12) {
        g_Pin2PortMapArray[12].GPIOx_Port->BSRR = (g_Pin2PortMapArray[12].Pin_abstraction << 16);
      } else if (pin == 13) {
        g_Pin2PortMapArray[13].GPIOx_Port->BSRR = (g_Pin2PortMapArray[13].Pin_abstraction << 16);
      } else if (pin == 14) {
        g_Pin2PortMapArray[14].GPIOx_Port->BSRR = (g_Pin2PortMapArray[14].Pin_abstraction << 16);
      } else if (pin == 15) {
        g_Pin2PortMapArray[15].GPIOx_Port->BSRR = (g_Pin2PortMapArray[15].Pin_abstraction << 16);
      } else if (pin == 16) {
        g_Pin2PortMapArray[16].GPIOx_Port->BSRR = (g_Pin2PortMapArray[16].Pin_abstraction << 16);
      } else if (pin == 17) {
        g_Pin2PortMapArray[17].GPIOx_Port->BSRR = (g_Pin2PortMapArray[17].Pin_abstraction << 16);
      } else if (pin == 18) {
        g_Pin2PortMapArray[18].GPIOx_Port->BSRR = (g_Pin2PortMapArray[18].Pin_abstraction << 16);
      } else if (pin == 19) {
        g_Pin2PortMapArray[19].GPIOx_Port->BSRR = (g_Pin2PortMapArray[19].Pin_abstraction << 16);
      } else if (pin == 20) {
        g_Pin2PortMapArray[20].GPIOx_Port->BSRR = (g_Pin2PortMapArray[20].Pin_abstraction << 16);
      } else if (pin == 10) {
        g_Pin2PortMapArray[10].GPIOx_Port->BSRR = (g_Pin2PortMapArray[10].Pin_abstraction << 16);
      } else if (pin == 11) {
        g_Pin2PortMapArray[11].GPIOx_Port->BSRR = (g_Pin2PortMapArray[11].Pin_abstraction << 16);
      } else if (pin == 12) {
        g_Pin2PortMapArray[12].GPIOx_Port->BSRR = (g_Pin2PortMapArray[12].Pin_abstraction << 16);
      } else if (pin == 13) {
        g_Pin2PortMapArray[13].GPIOx_Port->BSRR = (g_Pin2PortMapArray[13].Pin_abstraction << 16);
      } else if (pin == 14) {
        g_Pin2PortMapArray[14].GPIOx_Port->BSRR = (g_Pin2PortMapArray[14].Pin_abstraction << 16);
      } else if (pin == 15) {
        g_Pin2PortMapArray[15].GPIOx_Port->BSRR = (g_Pin2PortMapArray[15].Pin_abstraction << 16);
      } else if (pin == 16) {
        g_Pin2PortMapArray[16].GPIOx_Port->BSRR = (g_Pin2PortMapArray[16].Pin_abstraction << 16);
      } else if (pin == 17) {
        g_Pin2PortMapArray[17].GPIOx_Port->BSRR = (g_Pin2PortMapArray[17].Pin_abstraction << 16);
      } else if (pin == 18) {
        g_Pin2PortMapArray[18].GPIOx_Port->BSRR = (g_Pin2PortMapArray[18].Pin_abstraction << 16);
      } else if (pin == 19) {
        g_Pin2PortMapArray[19].GPIOx_Port->BSRR = (g_Pin2PortMapArray[19].Pin_abstraction << 16);
      } else if (pin == 20) {
        g_Pin2PortMapArray[20].GPIOx_Port->BSRR = (g_Pin2PortMapArray[20].Pin_abstraction << 16);
      } else if (pin == 21) {
        g_Pin2PortMapArray[21].GPIOx_Port->BSRR = (g_Pin2PortMapArray[21].Pin_abstraction << 16);
      } else if (pin == 22) {
        g_Pin2PortMapArray[22].GPIOx_Port->BSRR = (g_Pin2PortMapArray[22].Pin_abstraction << 16);
      } else if (pin == 23) {
        g_Pin2PortMapArray[23].GPIOx_Port->BSRR = (g_Pin2PortMapArray[23].Pin_abstraction << 16);
      } else if (pin == 24) {
        g_Pin2PortMapArray[24].GPIOx_Port->BSRR = (g_Pin2PortMapArray[24].Pin_abstraction << 16);
      } else if (pin == 25) {
        g_Pin2PortMapArray[25].GPIOx_Port->BSRR = (g_Pin2PortMapArray[25].Pin_abstraction << 16);
      } else if (pin == 26) {
        g_Pin2PortMapArray[26].GPIOx_Port->BSRR = (g_Pin2PortMapArray[26].Pin_abstraction << 16);
      } else if (pin == 27) {
        g_Pin2PortMapArray[27].GPIOx_Port->BSRR = (g_Pin2PortMapArray[27].Pin_abstraction << 16);
      } else if (pin == 28) {
        g_Pin2PortMapArray[28].GPIOx_Port->BSRR = (g_Pin2PortMapArray[28].Pin_abstraction << 16);
      } else if (pin == 29) {
        g_Pin2PortMapArray[29].GPIOx_Port->BSRR = (g_Pin2PortMapArray[29].Pin_abstraction << 16);
      } else if (pin == 30) {
        g_Pin2PortMapArray[30].GPIOx_Port->BSRR = (g_Pin2PortMapArray[30].Pin_abstraction << 16);
      } else if (pin == 31) {
        g_Pin2PortMapArray[31].GPIOx_Port->BSRR = (g_Pin2PortMapArray[31].Pin_abstraction << 16);
      } else if (pin == 32) {
        g_Pin2PortMapArray[32].GPIOx_Port->BSRR = (g_Pin2PortMapArray[32].Pin_abstraction << 16);
      } else if (pin == 33) {
        g_Pin2PortMapArray[33].GPIOx_Port->BSRR = (g_Pin2PortMapArray[33].Pin_abstraction << 16);
      } else if (pin == 34) {
        g_Pin2PortMapArray[34].GPIOx_Port->BSRR = (g_Pin2PortMapArray[34].Pin_abstraction << 16);
      } else if (pin == 35) {
        g_Pin2PortMapArray[35].GPIOx_Port->BSRR = (g_Pin2PortMapArray[35].Pin_abstraction << 16);
      } else if (pin == 36) {
        g_Pin2PortMapArray[36].GPIOx_Port->BSRR = (g_Pin2PortMapArray[36].Pin_abstraction << 16);
      } else if (pin == 37) {
        g_Pin2PortMapArray[37].GPIOx_Port->BSRR = (g_Pin2PortMapArray[37].Pin_abstraction << 16);
      } else if (pin == 38) {
        g_Pin2PortMapArray[38].GPIOx_Port->BSRR = (g_Pin2PortMapArray[38].Pin_abstraction << 16);
      } else if (pin == 39) {
        g_Pin2PortMapArray[39].GPIOx_Port->BSRR = (g_Pin2PortMapArray[39].Pin_abstraction << 16);
      } else if (pin == 40) {
        g_Pin2PortMapArray[40].GPIOx_Port->BSRR = (g_Pin2PortMapArray[40].Pin_abstraction << 16);
      } else if (pin == 41) {
        g_Pin2PortMapArray[41].GPIOx_Port->BSRR = (g_Pin2PortMapArray[41].Pin_abstraction << 16);
      } else if (pin == 42) {
        g_Pin2PortMapArray[42].GPIOx_Port->BSRR = (g_Pin2PortMapArray[42].Pin_abstraction << 16);
      } else if (pin == 43) {
        g_Pin2PortMapArray[43].GPIOx_Port->BSRR = (g_Pin2PortMapArray[43].Pin_abstraction << 16);
      } else if (pin == 44) {
        g_Pin2PortMapArray[44].GPIOx_Port->BSRR = (g_Pin2PortMapArray[44].Pin_abstraction << 16);
      } else if (pin == 45) {
        g_Pin2PortMapArray[45].GPIOx_Port->BSRR = (g_Pin2PortMapArray[45].Pin_abstraction << 16);
      } else if (pin == 46) {
        g_Pin2PortMapArray[46].GPIOx_Port->BSRR = (g_Pin2PortMapArray[46].Pin_abstraction << 16);
      } else if (pin == 47) {
        g_Pin2PortMapArray[47].GPIOx_Port->BSRR = (g_Pin2PortMapArray[47].Pin_abstraction << 16);
      } else if (pin == 48) {
        g_Pin2PortMapArray[48].GPIOx_Port->BSRR = (g_Pin2PortMapArray[48].Pin_abstraction << 16);
      } else if (pin == 49) {
        g_Pin2PortMapArray[49].GPIOx_Port->BSRR = (g_Pin2PortMapArray[49].Pin_abstraction << 16);
      } else if (pin == 50) {
        g_Pin2PortMapArray[50].GPIOx_Port->BSRR = (g_Pin2PortMapArray[50].Pin_abstraction << 16);
      } else if (pin == 51) {
        g_Pin2PortMapArray[51].GPIOx_Port->BSRR = (g_Pin2PortMapArray[51].Pin_abstraction << 16);
      } else if (pin == 52) {
        g_Pin2PortMapArray[52].GPIOx_Port->BSRR = (g_Pin2PortMapArray[52].Pin_abstraction << 16);
      } else if (pin == 53) {
        g_Pin2PortMapArray[53].GPIOx_Port->BSRR = (g_Pin2PortMapArray[53].Pin_abstraction << 16);
      } else if (pin == 54) {
        g_Pin2PortMapArray[54].GPIOx_Port->BSRR = (g_Pin2PortMapArray[54].Pin_abstraction << 16);
      } else if (pin == 55) {
        g_Pin2PortMapArray[55].GPIOx_Port->BSRR = (g_Pin2PortMapArray[55].Pin_abstraction << 16);
      } else if (pin == 56) {
        g_Pin2PortMapArray[56].GPIOx_Port->BSRR = (g_Pin2PortMapArray[56].Pin_abstraction << 16);
      } else if (pin == 57) {
        g_Pin2PortMapArray[57].GPIOx_Port->BSRR = (g_Pin2PortMapArray[57].Pin_abstraction << 16);
      } else if (pin == 58) {
        g_Pin2PortMapArray[58].GPIOx_Port->BSRR = (g_Pin2PortMapArray[58].Pin_abstraction << 16);
      } else if (pin == 59) {
        g_Pin2PortMapArray[59].GPIOx_Port->BSRR = (g_Pin2PortMapArray[59].Pin_abstraction << 16);
      } else if (pin == 60) {
        g_Pin2PortMapArray[60].GPIOx_Port->BSRR = (g_Pin2PortMapArray[60].Pin_abstraction << 16);
      } else if (pin == 61) {
        g_Pin2PortMapArray[61].GPIOx_Port->BSRR = (g_Pin2PortMapArray[61].Pin_abstraction << 16);
      } else if (pin == 62) {
        g_Pin2PortMapArray[62].GPIOx_Port->BSRR = (g_Pin2PortMapArray[62].Pin_abstraction << 16);
      } else if (pin == 63) {
        g_Pin2PortMapArray[63].GPIOx_Port->BSRR = (g_Pin2PortMapArray[63].Pin_abstraction << 16);
      } else if (pin == 64) {
        g_Pin2PortMapArray[64].GPIOx_Port->BSRR = (g_Pin2PortMapArray[64].Pin_abstraction << 16);
      } else if (pin == 65) {
        g_Pin2PortMapArray[65].GPIOx_Port->BSRR = (g_Pin2PortMapArray[65].Pin_abstraction << 16);
      } else if (pin == 66) {
        g_Pin2PortMapArray[66].GPIOx_Port->BSRR = (g_Pin2PortMapArray[66].Pin_abstraction << 16);
      } else if (pin == 67) {
        g_Pin2PortMapArray[67].GPIOx_Port->BSRR = (g_Pin2PortMapArray[67].Pin_abstraction << 16);
      } else if (pin == 68) {
        g_Pin2PortMapArray[68].GPIOx_Port->BSRR = (g_Pin2PortMapArray[68].Pin_abstraction << 16);
      } else if (pin == 69) {
        g_Pin2PortMapArray[69].GPIOx_Port->BSRR = (g_Pin2PortMapArray[69].Pin_abstraction << 16);
      } else if (pin == 70) {
        g_Pin2PortMapArray[70].GPIOx_Port->BSRR = (g_Pin2PortMapArray[70].Pin_abstraction << 16);
      } else if (pin == 71) {
        g_Pin2PortMapArray[71].GPIOx_Port->BSRR = (g_Pin2PortMapArray[71].Pin_abstraction << 16);
      } else if (pin == 72) {
        g_Pin2PortMapArray[72].GPIOx_Port->BSRR = (g_Pin2PortMapArray[72].Pin_abstraction << 16);
      } else if (pin == 73) {
        g_Pin2PortMapArray[73].GPIOx_Port->BSRR = (g_Pin2PortMapArray[73].Pin_abstraction << 16);
      } else if (pin == 74) {
        g_Pin2PortMapArray[74].GPIOx_Port->BSRR = (g_Pin2PortMapArray[74].Pin_abstraction << 16);
      } else if (pin == 75) {
        g_Pin2PortMapArray[75].GPIOx_Port->BSRR = (g_Pin2PortMapArray[75].Pin_abstraction << 16);
      } else if (pin == 76) {
        g_Pin2PortMapArray[76].GPIOx_Port->BSRR = (g_Pin2PortMapArray[76].Pin_abstraction << 16);
      } else if (pin == 77) {
        g_Pin2PortMapArray[77].GPIOx_Port->BSRR = (g_Pin2PortMapArray[77].Pin_abstraction << 16);
      } else if (pin == 78) {
        g_Pin2PortMapArray[78].GPIOx_Port->BSRR = (g_Pin2PortMapArray[78].Pin_abstraction << 16);
      } else if (pin == 79) {
        g_Pin2PortMapArray[79].GPIOx_Port->BSRR = (g_Pin2PortMapArray[89].Pin_abstraction << 16);
      } else if (pin == 80) {
        g_Pin2PortMapArray[80].GPIOx_Port->BSRR = (g_Pin2PortMapArray[80].Pin_abstraction << 16);
      } else if (pin == 81) {
        g_Pin2PortMapArray[81].GPIOx_Port->BSRR = (g_Pin2PortMapArray[81].Pin_abstraction << 16);
      } else if (pin == 82) {
        g_Pin2PortMapArray[82].GPIOx_Port->BSRR = (g_Pin2PortMapArray[82].Pin_abstraction << 16);
      } else if (pin == 83) {
        g_Pin2PortMapArray[83].GPIOx_Port->BSRR = (g_Pin2PortMapArray[83].Pin_abstraction << 16);
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
    if (pin == 0) {
      return (g_Pin2PortMapArray[0].GPIOx_Port->IDR & g_Pin2PortMapArray[0].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 1) {
      return (g_Pin2PortMapArray[1].GPIOx_Port->IDR & g_Pin2PortMapArray[1].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 2) {
      return (g_Pin2PortMapArray[2].GPIOx_Port->IDR & g_Pin2PortMapArray[2].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 3) {
      return (g_Pin2PortMapArray[3].GPIOx_Port->IDR & g_Pin2PortMapArray[3].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 4) {
      return (g_Pin2PortMapArray[4].GPIOx_Port->IDR & g_Pin2PortMapArray[4].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 5) {
      return (g_Pin2PortMapArray[5].GPIOx_Port->IDR & g_Pin2PortMapArray[5].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 6) {
      return (g_Pin2PortMapArray[6].GPIOx_Port->IDR & g_Pin2PortMapArray[6].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 7) {
      return (g_Pin2PortMapArray[7].GPIOx_Port->IDR & g_Pin2PortMapArray[7].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 8) {
      return (g_Pin2PortMapArray[8].GPIOx_Port->IDR & g_Pin2PortMapArray[8].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 9) {
      return (g_Pin2PortMapArray[9].GPIOx_Port->IDR & g_Pin2PortMapArray[9].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 10) {
      return (g_Pin2PortMapArray[10].GPIOx_Port->IDR & g_Pin2PortMapArray[10].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 11) {
      return (g_Pin2PortMapArray[11].GPIOx_Port->IDR & g_Pin2PortMapArray[11].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 12) {
      return (g_Pin2PortMapArray[12].GPIOx_Port->IDR & g_Pin2PortMapArray[12].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 13) {
      return (g_Pin2PortMapArray[13].GPIOx_Port->IDR & g_Pin2PortMapArray[13].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 14) {
      return (g_Pin2PortMapArray[14].GPIOx_Port->IDR & g_Pin2PortMapArray[14].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 15) {
      return (g_Pin2PortMapArray[15].GPIOx_Port->IDR & g_Pin2PortMapArray[15].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 16) {
      return (g_Pin2PortMapArray[16].GPIOx_Port->IDR & g_Pin2PortMapArray[16].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 17) {
      return (g_Pin2PortMapArray[17].GPIOx_Port->IDR & g_Pin2PortMapArray[17].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 18) {
      return (g_Pin2PortMapArray[18].GPIOx_Port->IDR & g_Pin2PortMapArray[18].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 19) {
      return (g_Pin2PortMapArray[19].GPIOx_Port->IDR & g_Pin2PortMapArray[19].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 20) {
      return (g_Pin2PortMapArray[20].GPIOx_Port->IDR & g_Pin2PortMapArray[20].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 21) {
      return (g_Pin2PortMapArray[21].GPIOx_Port->IDR & g_Pin2PortMapArray[21].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 22) {
      return (g_Pin2PortMapArray[22].GPIOx_Port->IDR & g_Pin2PortMapArray[22].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 23) {
      return (g_Pin2PortMapArray[23].GPIOx_Port->IDR & g_Pin2PortMapArray[23].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 24) {
      return (g_Pin2PortMapArray[24].GPIOx_Port->IDR & g_Pin2PortMapArray[24].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 25) {
      return (g_Pin2PortMapArray[25].GPIOx_Port->IDR & g_Pin2PortMapArray[25].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 26) {
      return (g_Pin2PortMapArray[26].GPIOx_Port->IDR & g_Pin2PortMapArray[26].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 27) {
      return (g_Pin2PortMapArray[27].GPIOx_Port->IDR & g_Pin2PortMapArray[27].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 28) {
      return (g_Pin2PortMapArray[28].GPIOx_Port->IDR & g_Pin2PortMapArray[28].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 29) {
      return (g_Pin2PortMapArray[29].GPIOx_Port->IDR & g_Pin2PortMapArray[29].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 30) {
      return (g_Pin2PortMapArray[30].GPIOx_Port->IDR & g_Pin2PortMapArray[30].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 31) {
      return (g_Pin2PortMapArray[31].GPIOx_Port->IDR & g_Pin2PortMapArray[31].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 32) {
      return (g_Pin2PortMapArray[32].GPIOx_Port->IDR & g_Pin2PortMapArray[32].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 33) {
      return (g_Pin2PortMapArray[33].GPIOx_Port->IDR & g_Pin2PortMapArray[33].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 34) {
      return (g_Pin2PortMapArray[34].GPIOx_Port->IDR & g_Pin2PortMapArray[34].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 35) {
      return (g_Pin2PortMapArray[35].GPIOx_Port->IDR & g_Pin2PortMapArray[35].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 36) {
      return (g_Pin2PortMapArray[36].GPIOx_Port->IDR & g_Pin2PortMapArray[36].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 37) {
      return (g_Pin2PortMapArray[37].GPIOx_Port->IDR & g_Pin2PortMapArray[37].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 38) {
      return (g_Pin2PortMapArray[38].GPIOx_Port->IDR & g_Pin2PortMapArray[38].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 39) {
      return (g_Pin2PortMapArray[39].GPIOx_Port->IDR & g_Pin2PortMapArray[39].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 30) {
      return (g_Pin2PortMapArray[40].GPIOx_Port->IDR & g_Pin2PortMapArray[40].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 41) {
      return (g_Pin2PortMapArray[41].GPIOx_Port->IDR & g_Pin2PortMapArray[41].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 42) {
      return (g_Pin2PortMapArray[42].GPIOx_Port->IDR & g_Pin2PortMapArray[42].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 43) {
      return (g_Pin2PortMapArray[43].GPIOx_Port->IDR & g_Pin2PortMapArray[43].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 44) {
      return (g_Pin2PortMapArray[44].GPIOx_Port->IDR & g_Pin2PortMapArray[44].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 45) {
      return (g_Pin2PortMapArray[45].GPIOx_Port->IDR & g_Pin2PortMapArray[45].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 46) {
      return (g_Pin2PortMapArray[46].GPIOx_Port->IDR & g_Pin2PortMapArray[46].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 47) {
      return (g_Pin2PortMapArray[47].GPIOx_Port->IDR & g_Pin2PortMapArray[47].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 48) {
      return (g_Pin2PortMapArray[48].GPIOx_Port->IDR & g_Pin2PortMapArray[48].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 49) {
      return (g_Pin2PortMapArray[49].GPIOx_Port->IDR & g_Pin2PortMapArray[49].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 50) {
      return (g_Pin2PortMapArray[50].GPIOx_Port->IDR & g_Pin2PortMapArray[50].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 51) {
      return (g_Pin2PortMapArray[51].GPIOx_Port->IDR & g_Pin2PortMapArray[51].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 52) {
      return (g_Pin2PortMapArray[52].GPIOx_Port->IDR & g_Pin2PortMapArray[52].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 53) {
      return (g_Pin2PortMapArray[53].GPIOx_Port->IDR & g_Pin2PortMapArray[53].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 54) {
      return (g_Pin2PortMapArray[54].GPIOx_Port->IDR & g_Pin2PortMapArray[54].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 55) {
      return (g_Pin2PortMapArray[55].GPIOx_Port->IDR & g_Pin2PortMapArray[55].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 56) {
      return (g_Pin2PortMapArray[56].GPIOx_Port->IDR & g_Pin2PortMapArray[56].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 57) {
      return (g_Pin2PortMapArray[57].GPIOx_Port->IDR & g_Pin2PortMapArray[57].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 58) {
      return (g_Pin2PortMapArray[58].GPIOx_Port->IDR & g_Pin2PortMapArray[58].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 59) {
      return (g_Pin2PortMapArray[59].GPIOx_Port->IDR & g_Pin2PortMapArray[59].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 60) {
      return (g_Pin2PortMapArray[60].GPIOx_Port->IDR & g_Pin2PortMapArray[60].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 61) {
      return (g_Pin2PortMapArray[61].GPIOx_Port->IDR & g_Pin2PortMapArray[61].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 62) {
      return (g_Pin2PortMapArray[62].GPIOx_Port->IDR & g_Pin2PortMapArray[62].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 63) {
      return (g_Pin2PortMapArray[63].GPIOx_Port->IDR & g_Pin2PortMapArray[63].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 64) {
      return (g_Pin2PortMapArray[64].GPIOx_Port->IDR & g_Pin2PortMapArray[64].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 65) {
      return (g_Pin2PortMapArray[65].GPIOx_Port->IDR & g_Pin2PortMapArray[65].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 66) {
      return (g_Pin2PortMapArray[66].GPIOx_Port->IDR & g_Pin2PortMapArray[66].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 67) {
      return (g_Pin2PortMapArray[67].GPIOx_Port->IDR & g_Pin2PortMapArray[67].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 68) {
      return (g_Pin2PortMapArray[68].GPIOx_Port->IDR & g_Pin2PortMapArray[68].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 69) {
      return (g_Pin2PortMapArray[69].GPIOx_Port->IDR & g_Pin2PortMapArray[69].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 70) {
      return (g_Pin2PortMapArray[70].GPIOx_Port->IDR & g_Pin2PortMapArray[70].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 71) {
      return (g_Pin2PortMapArray[71].GPIOx_Port->IDR & g_Pin2PortMapArray[71].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 72) {
      return (g_Pin2PortMapArray[72].GPIOx_Port->IDR & g_Pin2PortMapArray[72].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 73) {
      return (g_Pin2PortMapArray[73].GPIOx_Port->IDR & g_Pin2PortMapArray[73].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 74) {
      return (g_Pin2PortMapArray[74].GPIOx_Port->IDR & g_Pin2PortMapArray[74].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 75) {
      return (g_Pin2PortMapArray[75].GPIOx_Port->IDR & g_Pin2PortMapArray[75].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 76) {
      return (g_Pin2PortMapArray[76].GPIOx_Port->IDR & g_Pin2PortMapArray[76].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 77) {
      return (g_Pin2PortMapArray[77].GPIOx_Port->IDR & g_Pin2PortMapArray[77].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 78) {
      return (g_Pin2PortMapArray[78].GPIOx_Port->IDR & g_Pin2PortMapArray[78].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 79) {
      return (g_Pin2PortMapArray[79].GPIOx_Port->IDR & g_Pin2PortMapArray[79].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 80) {
      return (g_Pin2PortMapArray[80].GPIOx_Port->IDR & g_Pin2PortMapArray[80].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 81) {
      return (g_Pin2PortMapArray[81].GPIOx_Port->IDR & g_Pin2PortMapArray[81].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 82) {
      return (g_Pin2PortMapArray[82].GPIOx_Port->IDR & g_Pin2PortMapArray[82].Pin_abstraction)? HIGH : LOW;
    } else if (pin == 83) {
      return (g_Pin2PortMapArray[83].GPIOx_Port->IDR & g_Pin2PortMapArray[83].Pin_abstraction)? HIGH : LOW;
    }
  } else {
    return digitalRead(pin);
  }
}

#endif /* _DIGITAL_WRITE_FAST_ */