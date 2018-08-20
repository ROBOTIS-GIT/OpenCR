/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../include/open_manipulator/OMDebug.h"

void OpenUSBSerialPort()
{
  USB.begin(57600);
}

void OpenDEBUGSerialPort()
{
  DEBUG.begin(57600);
}

void LOG::log(String form, String msg, String port)
{
  if (port == "USB")
    USB.println(form + msg);
  else if (port == "DEBUG")
    DEBUG.println(form + msg);
}

void LOG::INFO(String msg)
{
  log("[INFO] ", msg);
}

void LOG::INFO(String msg, float num, uint8_t point, String port)
{
  if (port == "USB")
  {
    USB.print("[INFO] ");
    USB.print(msg);
    USB.println(num, point);
  }
  else if (port == "DEBUG")
  {
    DEBUG.print("[INFO] ");
    DEBUG.print(msg);
    DEBUG.println(num, point);
  }
}

void LOG::WARN(String msg)
{
  log("[WARN] ", msg);
}

void LOG::WARN(String msg, float num, uint8_t point, String port)
{
  if (port == "USB")
  {
    USB.print("[WARN] ");
    USB.print(msg);
    USB.println(num, point);
  }
  else if (port == "DEBUG")
  {
    DEBUG.print("[WARN] ");
    DEBUG.print(msg);
    DEBUG.println(num, point);
  }
}

void LOG::ERROR(String msg)
{
  log("[ERROR] ", msg);
}

void LOG::ERROR(String msg, float num, uint8_t point, String port)
{
  if (port == "USB")
  {
    USB.print("[ERROR] ");
    USB.print(msg);
    USB.println(num, point);
  }
  else if (port == "DEBUG")
  {
    DEBUG.print("[ERROR] ");
    DEBUG.print(msg);
    DEBUG.println(num, point);
  }
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;


  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(0);
    }
    else
    {
      setLedOff(0);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(1);
    }
    else
    {
      setLedOff(1);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500 )
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(2);
  }
  else
  {
    setLedOff(2);
  }

  if (getUsbConnected() > 0)
  {
    setLedOn(3);
  }
  else
  {
    setLedOff(3);
  }

  updateRxTxLed();
}

// void showJointAngle(String unit, OPMLink* link, int from, int to)
// {
//   int num = 0;

//   if (unit == "rad")
//   {
//     for (num = from; num <= to; num++)
//     {
//       Serial.print(link[num].joint_angle_);
//       Serial.print(" ");
//     }
//     Serial.println("");
//   }
//   else if (unit == "deg")
//   {
//     for (num = from; num <= to; num++)
//     {
//       Serial.print(link[num].joint_angle_*RAD2DEG);
//       Serial.print(" ");
//     }
//     Serial.println("");
//   }
// }

// void showFKResult(OPMLink* link, int from, int to)
// {
//   int num = 0;

//   for (num = from; num <= to; num++)
//   {
//     Serial.print("link : "); Serial.println(link[num].name_);
//     Serial.println("p_ : "); print_vt3f(link[num].p_);
//     Serial.println("R_ : "); print_mt3f(link[num].R_);
//   }
// }