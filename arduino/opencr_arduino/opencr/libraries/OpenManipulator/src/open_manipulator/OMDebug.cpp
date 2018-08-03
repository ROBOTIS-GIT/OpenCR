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

void LOG::log(String form, String msg)
{
  USB.println(form + msg);
}

void LOG::INFO(String msg)
{
  log("[INFO] ", msg);
}

void LOG::WARN(String msg)
{
  log("[WARN] ", msg);
}

void LOG::ERROR(String msg)
{
  log("[ERROR] ", msg);
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