/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef REMOTECONTROLLER100_H_
#define REMOTECONTROLLER100_H_

#include <linear_libs.h>
#include <RC100.h>
#include "demo.h"

RC100 rc100;

/*****************************************************************************
** Initialize baudrate for using RC100
*****************************************************************************/
void initRC100()
{
  rc100.begin(1); // Using Serial2(=SerialBT1)
}

/*****************************************************************************
** Receive data from RC100
*****************************************************************************/
void receiveDataFromRC100(Linear* linear)
{
  if (!linear->getReceiveDataFlag())
  {
    if (rc100.available())
    {
      // Read data received from RC100
      uint16_t data = rc100.readData();
      if (data & RC100_BTN_U)
        linear->makeTaskTrajectory("tool", math::vector3(0.006, 0.0, 0.0), 0.2);
      else if (data & RC100_BTN_D)
        linear->makeTaskTrajectory("tool", math::vector3(-0.006, 0.0, 0.0), 0.2);
      else if (data & RC100_BTN_L)
        linear->makeTaskTrajectory("tool", math::vector3(0.0, 0.006, 0.0), 0.2);
      else if (data & RC100_BTN_R)
        linear->makeTaskTrajectory("tool", math::vector3(0.0, -0.006, 0.0), 0.2);
      else if (data & RC100_BTN_1)
        linear->makeToolTrajectory("tool", -0.007);
      else if (data & RC100_BTN_2)
        linear->makeToolTrajectory("tool", 0.007);
      else if (data & RC100_BTN_3)
        startDemo();
      else if (data & RC100_BTN_4)
        stopDemo(linear);
      else if (data & RC100_BTN_5) {}
      else if (data & RC100_BTN_6)
      {
        std::vector<double> target_angle;
        target_angle.push_back(0.0);
        target_angle.push_back(0.0);
        target_angle.push_back(0.0);
        linear->makeJointTrajectory(target_angle, 1.0);
      }
      
//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      linear->setReceiveDataFlag(true);
      linear->setPrevReceiveTime(millis()/1000.0);
    }
  }
  else
  {
    if (millis()/1000.0 - linear->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      linear->setReceiveDataFlag(false);
      initRC100();
    }
  }
}
#endif // REMOTECONTROLLER100_H_
