/*
 * blink_led.cpp
 *
 *  Created on: 2017. 3. 11.
 *      Author: Baram
 */

#include "Arduino.h"




void setup(void)
{
  Serial.begin(115200);

  pinMode(13, OUTPUT);
}


void loop(void)
{
  static uint32_t t_time[8];


  if (millis()-t_time[0] >= 1000)
  {
    Serial.println("Test");
    digitalWrite(13, !digitalRead(13));
    t_time[0] = millis();
  }

  if (Serial.available())
  {
    Serial.print("Received : 0x");
    Serial.println(Serial.read(), HEX);
  }
}


