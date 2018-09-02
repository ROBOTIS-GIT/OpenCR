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

  if (cmd[0] == "opm")
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
    
    for (uint8_t index = 0; index < ACTIVE_JOINT_SIZE; index++)
    {
      goal_position.push_back(cmd[index+1].toFloat());
    }

    SCARA.jointMove(goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "tool")
  {
    SCARA.toolMove(TOOL, cmd[1].toFloat());
  }
  else if (cmd[0] == "tool")
  {
    if (cmd[1] == "on")
      SCARA.toolMove(TOOL, 0.0f);
    else if (cmd[1] == "off")
      SCARA.toolMove(TOOL, -1.0f);
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
  // else if (cmd[0] == "get")
  // {
  //   if (cmd[1] == "on")
  //   {
  //     motion_storage[motion_num][0] = 3.0;  //mov_time
  //     motion_storage[motion_num][1] = -1.0;
  //     motion_num++;
  //   }
  //   else if (cmd[1] == "off")
  //   {
  //     motion_storage[motion_num][0] = 3.0;  //mov_time
  //     motion_storage[motion_num][1] = 1.0;
  //     motion_num++;
  //   }
  //   else if (cmd[1] == "clear")
  //   {
  //     for (int i = 0; i < MOTION_NUM; i++)
  //     {
  //       for (int j = 0; j < 6; j++)
  //       {
  //         motion_storage[i][j] = 0.0;
  //       }
  //     }
      
  //     motion_num = 0;
  //     motion_cnt = 0;
  //     motion     = false;
  //     repeat     = false;
  //   }
  //   else if (cmd[1] == "pose")
  //   {
  //     if (cmd[2].toInt() < MOTION_NUM)
  //     {
  //       State* state = getAngle();

  //       if (DYNAMIXEL)
  //         sendAngle2Processing(state); 

  //       for (int i = JOINT1; i < GRIP; i++)
  //       {
  //         motion_storage[motion_num][0]   = 3.0;  //mov_time
  //         motion_storage[motion_num][i+1] = state[i].pos;
  //       }
  //       motion_num++;
  //     }
  //   }
  // }
  // else if (cmd[0] == "hand")
  // {
  //   if (cmd[1] == "once")
  //   {
  //     if (DYNAMIXEL)
  //     {
  //       setTorque(true);
  //       sendAngle2Processing(getAngle()); 
  //     }

  //     motion_cnt = 0;
  //     motion = true;
  //   }
  //   else if (cmd[1] == "repeat")
  //   {
  //     if (DYNAMIXEL)
  //     {
  //       setTorque(true);
  //       sendAngle2Processing(getAngle()); 
  //     }

  //     motion_cnt = 0;
  //     motion = true;
  //     repeat = true;
  //   }
  //   else if (cmd[1] == "stop")
  //   {
  //     for (int i = 0; i < MOTION_NUM; i++)
  //     {
  //       for (int j = 0; j < 6; j++)
  //       {
  //         motion_storage[i][j] = 0;
  //       }
  //     }

  //     motion_cnt = 0;
  //     motion     = false;
  //     repeat     = false;
  //   }
  // }
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {

#if 0 // DRAW CIRCLE
      const float move_time = 1.0f;
      float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
      void *p_init_arg = init_arg;

      static float radius = 0.005f;

      SCARA.drawInit(CIRCLE, move_time, p_init_arg);
      SCARA.setRadiusForDrawing(CIRCLE, radius);  
      SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
      SCARA.draw(CIRCLE);

      radius += 0.005f;
#endif

      SCARA.drawLine(SCARA.getComponentPositionToWorld(TOOL), 
              (SCARA.getComponentPositionToWorld(TOOL) - OM_MATH::makeVector3(0.0, 0.010, 0.0)), 
              1.0f);

      // if (DYNAMIXEL)
      //   sendAngle2Processing(getAngle()); 

      // if (PROCESSING)
      //   sendAngle2Processing(getState()); 

      // for (int i = 0; i < MOTION_NUM; i++)
      // {
      //   for (int j = 0; j < 6; j++)
      //   {
      //     motion_storage[i][j] = motion_set[i][j];
      //   }
      // }

      // motion_num = MOTION_NUM;  
      // motion_cnt = 0;          
      // motion = true;
      // repeat = true;
    }
    else if (cmd[1] == "stop")
    {
      SCARA.drawLine(SCARA.getComponentPositionToWorld(TOOL), 
        (SCARA.getComponentPositionToWorld(TOOL) - OM_MATH::makeVector3(0.010, 0.0, 0.0)), 
        1.0f);

      // motion_cnt = 0;
      // motion     = false;
      // repeat     = false;
    }
  }
}
#endif