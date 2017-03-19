/*
 *  delay.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram, PBHP
 */

#include "delay.h"



extern void delayMicroseconds(uint32_t usec);



void delay_ns(uint32_t ns)
{
  for (volatile uint32_t i = 0; i < ns/10; i++) { }
}

void delay_us(uint32_t us)
{
  delayMicroseconds(us);
}

void delay_ms(uint32_t ms)
{
  volatile uint32_t t_start = millis();


  while (1)
  {
    volatile uint32_t t[2];
    // poll time twice in case we happen to poll during timer wrap glitch
    t[0] = millis();
    t[1] = millis();
    t[0] = t[1] < t[0] ? t[1] : t[0];
    if (t[0] > t_start + ms)
      break;
  }
}
