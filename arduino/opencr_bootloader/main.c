/*
 * OpenCR BootLoader Firmware.
 *
 * by Baram (http://oroca.org)
 * by
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "main.h"



void main_init();




int main(void)
{

  main_init();


  while(1)
  {
    led_toggle(0);
    delay_ms(1000);
  }
}


void main_init()
{
  hal_init();
}
