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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "Chain.h"

void connectProcessing()
{
  chain.connectProcessing(DXL_SIZE);
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
  String *cmd = chain.parseDataFromProcessing(data);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      chain.actuatorEnable();
      chain.sendAngleToProcessing(chain.receiveAllActuatorAngle());
      chain.sendToolData2Processing(chain.getComponentToolValue(TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      chain.actuatorDisable();
#endif
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<float> goal_position;

    for (uint8_t index = 0; index < ACTIVE_JOINT_SIZE; index++)
    {
      goal_position.push_back(cmd[index + 1].toFloat());
    }

    chain.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "gripper")
  {
    chain.toolMove(TOOL, cmd[1].toFloat());
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      chain.toolMove(TOOL, 0.0f);
    else if (cmd[1] == "off")
      chain.toolMove(TOOL, -1.0f);
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      chain.setMove(TOOL, OM_MATH::makeVector3(0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "back")
      chain.setMove(TOOL, OM_MATH::makeVector3(-0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "left")
      chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.010f, 0.0), 0.2f);
    else if (cmd[1] == "right")
      chain.setMove(TOOL, OM_MATH::makeVector3(0.0, -0.010f, 0.0), 0.2f);
    else if (cmd[1] == "up")
      chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.010f), 0.2f);
    else if (cmd[1] == "down")
      chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.010f), 0.2f);
    else
      chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.0), 0.2f);
  }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      chain.actuatorEnable();
    else if (cmd[1] == "off")
      chain.actuatorDisable();
#endif
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      chain.drawLine(TOOL, OM_MATH::makeVector3(0.050, 0.0, 0.050f), 0.5f);
    }
    else if (cmd[1] == "stop")
    {
      chain.drawLine(TOOL, OM_MATH::makeVector3(-0.050, 0.0, -0.050f), 0.5f);
    }
  }
}
#endif