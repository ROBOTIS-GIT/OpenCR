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

#include <delta_libs.h>
#include <RC100.h>
#include "demo.h"

RC100 rc100;
double grip_value = 0.0;

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
void receiveDataFromRC100(Delta* delta)
{
  if (!delta->getReceiveDataFlag())
  {
    if (rc100.available())
    {
      delta->setReceiveDataFlag(true);

      uint16_t data = rc100.readData();

      // Task space control tab 
      if (data & RC100_BTN_U) 
        delta->makeTaskTrajectory("tool", math::vector3(0.020, 0.0, 0.0), 0.1);
      else if (data & RC100_BTN_L) 
        delta->makeTaskTrajectory("tool", math::vector3(-0.020, 0.0, 0.0), 0.1);
      else if (data & RC100_BTN_D)
        delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.020, 0.0), 0.1);
      else if (data & RC100_BTN_R) 
        delta->makeTaskTrajectory("tool", math::vector3(0.0, -0.020, 0.0), 0.1);
      else if (data & RC100_BTN_1)
        delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.015), 0.1);
      else if (data & RC100_BTN_2)
        delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.015), 0.1);
      else if (data & RC100_BTN_3) 
        startDemo();
      else if (data & RC100_BTN_4)         
        stopDemo(delta);
      else if (data & RC100_BTN_5) {}
      else if (data & RC100_BTN_6)
      {
        std::vector<double> goal_position;
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        delta->makeJointTrajectory(goal_position, 1.0);
      }
      
//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      delta->setReceiveDataFlag(true);
      delta->setPrevReceiveTime(millis()/1000.0); 
    }
  }
  else 
  {
    if (millis()/1000.0 - delta->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      delta->setReceiveDataFlag(false);   
      initRC100();
    }
  }
}

#endif // REMOTECONTROLLER100_H_
