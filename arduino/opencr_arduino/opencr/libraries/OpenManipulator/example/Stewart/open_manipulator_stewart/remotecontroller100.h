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

#include <stewart_libs.h>
#include <RC100.h>
#include "demo.h"

RC100 rc100;

/*****************************************************************************
** Initialize baudrate for using RC100
*****************************************************************************/
void initRC100()
{
  rc100.begin(1); // using Serial2(=SerialBT1)
}

/*****************************************************************************
** Receive data from RC100
*****************************************************************************/
void receiveDataFromRC100(Stewart* stewart)
{
  if (!stewart->getReceiveDataFlag())
  {
    if (rc100.available())
    {
      stewart->setReceiveDataFlag(true);

      uint16_t data = rc100.readData();

      // Task space control tab 
      if (data & RC100_BTN_U) 
        stewart->makeTaskTrajectory("tool", math::vector3(0.020, 0.0, 0.0), 1.0);
      else if (data & RC100_BTN_L) 
        stewart->makeTaskTrajectory("tool", math::vector3(-0.020, 0.0, 0.0), 1.0);
      else if (data & RC100_BTN_D)
        stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.020, 0.0), 1.0);
      else if (data & RC100_BTN_R) 
        stewart->makeTaskTrajectory("tool", math::vector3(0.0, -0.020, 0.0), 1.0);
      else if (data & RC100_BTN_1)
        stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.015), 1.0);
      else if (data & RC100_BTN_2)
        stewart->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.015), 1.0);
      else if (data & RC100_BTN_3)
        startDemo();      
      else if (data & RC100_BTN_4)
        stopDemo(stewart);
      else if (data & RC100_BTN_5) {}
      else if (data & RC100_BTN_6)
      {
        std::vector<double> goal_position;
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        stewart->makeJointTrajectory(goal_position, 0.5);        
      }
      
//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      stewart->setReceiveDataFlag(true);
      stewart->setPrevReceiveTime(millis()/1000.0);
    }
  }
  else 
  {
    if (millis()/1000.0 - stewart->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      stewart->setReceiveDataFlag(false);   
      initRC100();
    }
  }
}

#endif // REMOTECONTROLLER100_H_
