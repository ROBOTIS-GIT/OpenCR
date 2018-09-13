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

/* Authors: Darby Lim, Ryan Shim, Hye-Jong KIM, Yong-Ho Na */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "SCARA.h"

void connectProcessing()
{
  SCARA.connectProcessing(DXL_SIZE);
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
  String *cmd = SCARA.parseDataFromProcessing(data);

  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      SCARA.actuatorEnable();
      SCARA.sendAngleToProcessing(SCARA.receiveAllActuatorAngle());  
      SCARA.sendToolData2Processing(SCARA.getComponentToolValue(TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      SCARA.actuatorDisable();
#endif
    }
  }
  else if (cmd[0] == "joint")
  {
    std::vector<float> goal_position;
    for (uint8_t index = 0; index < SCARA.getDOF(); index++)
    {
      goal_position.push_back(cmd[index+1].toFloat());
    }

    SCARA.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "move_tool")
  {
    SCARA.toolMove(TOOL, cmd[1].toFloat());
  }
  else if (cmd[0] == "tool")
  {
    if (cmd[1] == "on")
      SCARA.toolMove(TOOL, 0.0f);
    else if (cmd[1] == "off")
      SCARA.toolMove(TOOL, -0.5f);
  }
  else if (cmd[0] == "pos")
  {
    Pose goal_pose;
    goal_pose.position(0) = cmd[1].toFloat();
    goal_pose.position(1) = cmd[2].toFloat();
    goal_pose.position(2) = 0.057;
    goal_pose.orientation = Eigen::Matrix3f::Identity(3,3);

    SCARA.setPose(TOOL, goal_pose, 1.5f);
  }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      SCARA.actuatorEnable();
    else if (cmd[1] == "off")
      SCARA.actuatorDisable();
#endif
  }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      float start_angular_position = 0.0f;
      const float move_time = 1.0f;
      float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
      void *p_init_arg = init_arg;
      float radius = 0.020f;
       
      SCARA.drawInit(CIRCLE, move_time, p_init_arg);
      SCARA.setRadiusForDrawing(CIRCLE, radius);  
      SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
      SCARA.setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
      SCARA.draw(CIRCLE);

      // radius = 0.040f;          
      // SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
      // SCARA.setRadiusForDrawing(RHOMBUS, radius);  
      // SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
      // SCARA.setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
      // SCARA.draw(RHOMBUS);

      // radius = 0.050f;
      // SCARA.drawInit(HEART, move_time, p_init_arg);
      // SCARA.setRadiusForDrawing(HEART, radius);  
      // SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
      // SCARA.setStartAngularPositionForDrawing(HEART, start_angular_position);
      // SCARA.draw(HEART);
    }
    else if (cmd[1] == "stop")
    {

    }
  }
}
#endif