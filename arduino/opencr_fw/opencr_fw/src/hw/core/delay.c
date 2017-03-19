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
  HAL_Delay(ms);
}

