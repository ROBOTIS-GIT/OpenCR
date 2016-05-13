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
