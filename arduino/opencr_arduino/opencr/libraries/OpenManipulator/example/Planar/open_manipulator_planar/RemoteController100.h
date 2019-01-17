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

#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include <Planar.h>
#include <RC100.h>
#include "Demo.h"

RC100 rc100;
double grip_value = 0.0;

//---------------------------------------------------------------------------------------------------- 
/* Initialize baudrate for using RC100 */
void initRC100()
{
  rc100.begin(1); // Using Serial2(=SerialBT1)
}

//---------------------------------------------------------------------------------------------------- 
/* Receive data from RC100 */
void receiveDataFromRC100(Planar* planar)
{
  if (!planar->getReceiveDataFlag())
  {
    if (rc100.available())
    {
      uint16_t data = rc100.readData();

      // Task space control tab 
      if (data & RC100_BTN_U)
        planar->taskTrajectoryMoveToPresentPose("tool", RM_MATH::makeVector3(0.006, 0.0, 0.0), 0.16);
      else if (data & RC100_BTN_D)
        planar->taskTrajectoryMoveToPresentPose("tool", RM_MATH::makeVector3(-0.006, 0.0, 0.0), 0.16);
      else if (data & RC100_BTN_L)
        planar->taskTrajectoryMoveToPresentPose("tool", RM_MATH::makeVector3(0.0, 0.006, 0.0), 0.16);
      else if (data & RC100_BTN_R)
        planar->taskTrajectoryMoveToPresentPose("tool", RM_MATH::makeVector3(0.0, -0.006, 0.0), 0.16);
      else if (data & RC100_BTN_1)
        planar->toolMove("tool", 0.0);
      else if (data & RC100_BTN_2)
        planar->toolMove("tool", 1.0);
      else if (data & RC100_BTN_3){}
      else if (data & RC100_BTN_4){}
      else if (data & RC100_BTN_5){}
      else if (data & RC100_BTN_6)
      {
        std::vector<double> goal_position;
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);
        goal_position.push_back(0.0);

        planar->jointTrajectoryMove(goal_position, 1.0);
      }

      // ...
      planar->setReceiveDataFlag(true);
      planar->setPrevReceiveTime(millis()/1000.0); // instead of curr_time...?
    }
  }
  else 
  {
    // Serial.println(".");
    // Check if running demo now..
    if (planar->getRunDemoFlag())
    {
      runDemo(planar); 
    }

    // Check if ???
    else if (millis()/1000.0 - planar->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      planar->setReceiveDataFlag(false);   //received <--
      initRC100();
    }
  }
}


#endif // REMOTECONTROLLER100_H_