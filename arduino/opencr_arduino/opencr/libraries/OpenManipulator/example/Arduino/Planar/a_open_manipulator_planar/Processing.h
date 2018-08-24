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

/* Authors: Darby Lim */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "Planar.h"

void connectProcessing()
{
  planar.connectProcessing(DXL_SIZE);
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
  String *cmd = planar.parseDataFromProcessing(data);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      planar.actuatorEnable();
      planar.sendAngleToProcessing(planar.receiveAllActuatorAngle());  
      planar.sendToolData2Processing(planar.getComponentToolValue(TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      planar.actuatorDisable();
#endif
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<float> goal_position;
    
    for (uint8_t index = 0; index < ACTIVE_JOINT_SIZE; index++)
    {
      goal_position.push_back(cmd[index+1].toFloat());
    }

    planar.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      planar.setMove(TOOL, OM_MATH::makeVector3(0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "back")
      planar.setMove(TOOL, OM_MATH::makeVector3(-0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "left")
      planar.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.010f, 0.0), 0.2f);
    else if (cmd[1] == "right")
      planar.setMove(TOOL, OM_MATH::makeVector3(0.0, -0.010f, 0.0), 0.2f);
    else
      planar.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.0), 0.2f);
  }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      planar.actuatorEnable();
    else if (cmd[1] == "off")
      planar.actuatorDisable();
#endif
  }
}
#endif