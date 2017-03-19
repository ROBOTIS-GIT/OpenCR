/*
 * main.cpp
 *
 *  Created on: 2017. 3. 19.
 *      Author: baram
 */
#include "main.h"



void mainInit(void);



int main(void)
{
  uint32_t t_time;
  uint32_t t_time_us;
  uint32_t cnt = 0.;

  mainInit();


  t_time = millis();
  while(1)
  {
    if (millis()-t_time >= 100)
    {
      vcpPrintf("test %d, %d\r\n", cnt, micros()-t_time_us);
      cnt++;
      t_time    = millis();
      t_time_us = micros();

      ledToggle(cnt%4);
    }

    if (vcpAvailable())
    {
      vcpPrintf("Received : %c\r\n", vcpRead());
    }
  }

  return 0;
}

void mainInit(void)
{
  hwInit();
  driverInit();
  apInit();
}
