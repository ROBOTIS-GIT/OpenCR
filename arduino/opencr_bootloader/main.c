/*
 * OpenCR BootLoader Firmware.
 *
 * by Baram
 * by PBPH
 * by http://oroca.org
 */

#include "main.h"



void main_init();




int main(void)
{

  main_init();


  while(1)
  {
    led_toggle(0);
    delay_ms(500);
  }
}


void main_init()
{
  hal_init();
}
