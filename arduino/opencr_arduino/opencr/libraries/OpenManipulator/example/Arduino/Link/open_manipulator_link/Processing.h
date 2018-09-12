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

/* Authors: Hye-Jong KIM, Darby Lim, Ryan Shim, Yong-Ho Na */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "Link.h"
#include "Suction.h"

void connectProcessing()
{
  Link.connectProcessing(12);
}

int availableProcessing()
{
  return Serial.available();
}

String readProcessingData()
{
  return Serial.readStringUntil('\n');
}

void fromProcessing(String data)
{
  String *cmd = Link.parseDataFromProcessing(data);

////////////////////////Manipulator OnOff//////////////////////////
  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      Link.actuatorEnable();
      Link.sendAngleToProcessing(Link.getAllActiveJointAngle());  
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      Link.actuatorDisable();  
#endif
    }
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Joint Move/////////////////////////////
  else if (cmd[0] == "joint")
  {
    std::vector<float> target_angle;

    for (uint8_t i = 0; i < 3; i++)
    {
      target_angle.push_back(cmd[i+1].toFloat());
    }

    Link.jointMove(target_angle, MOVETIME);
    target_angle.clear();
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Suction OnOff///////////////////////////
  else if (cmd[0] == "suction")
  {
    if (cmd[1] == "on")
    {
      suctionOn();
    }
    else if (cmd[1] == "off")
    {
      suctionOff();
    }
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Task move//////////////////////////////
  else if (cmd[0] == "task")
  {
    Pose target_pose;
    std::vector<float> target_angle;

    if (cmd[1] == "forward")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(MOVESTEP, 0.0, 0.0), MOVETIME);
    }  
    else if (cmd[1] == "back")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(-MOVESTEP, 0.0, 0.0), MOVETIME);
    }
    else if (cmd[1] == "left")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(0.0, MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "right")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(0.0, -MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "up")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, MOVESTEP), MOVETIME);
    }
    else if (cmd[1] == "down")
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, -MOVESTEP), MOVETIME);
    }
    else
    {
      Link.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, 0.0), MOVETIME);
    }
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Motor onOff/////////////////////////////
  else if (cmd[0] == "motor")
  {
#ifdef PLATFORM
    if (cmd[1] == "enable")
      {
        Link.actuatorEnable();
      }
      else if (cmd[1] == "disable")
      {
        Link.actuatorDisable();
      }
#endif
    DEBUG.println(" ");
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {

    }
    else if (cmd[1] == "stop")
    {

    }
  }
}
////////////////////////////////////////////////////////////////////
#endif