/*
 * OpenCR BootLoader Firmware.
 *
 * by Baram
 * by PBPH
 * by http://oroca.org
 */

#include "main.h"



void main_init();



extern void CDC_Write( uint8_t *p_buf, uint32_t length );

int main(void)
{
  uint32_t tTime;
  uint8_t cnt = 0;
  uint8_t ch;


  main_init();

  tTime = millis();
  while(1)
  {
    if( millis()-tTime > 500 )
    {
      tTime = millis();
      led_toggle(0);
      vcp_printf("cnt : %d \r\n", cnt++);
    }

    if( vcp_is_available() )
    {
      ch = vcp_getch();
      vcp_printf("pressed : 0x%02X \r\n", ch);
    }
  }
}


void main_init()
{
  bsp_init();
  hal_init();
}
