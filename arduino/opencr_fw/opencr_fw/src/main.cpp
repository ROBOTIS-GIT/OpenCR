/*
 * main.cpp
 *
 *  Created on: 2017. 3. 19.
 *      Author: baram
 */
#include "main.h"



int main(void)
{
  uint32_t t_time;


  hwInit();
  driverInit();

  t_time = millis();
  while(1)
  {
    if (millis()-t_time > 100)
    {
      t_time = millis();
      vcpPrintf("test \r\n");
    }

    if (vcpIsAvailable())
    {
      vcpPrintf("Received : %c\r\n", vcpRead());
    }
  }

  return 0;
}
