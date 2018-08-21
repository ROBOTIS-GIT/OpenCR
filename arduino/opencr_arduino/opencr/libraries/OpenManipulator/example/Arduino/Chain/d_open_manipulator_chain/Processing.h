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

#include "Chain.h"

void connectProcessing()
{
  manipulator.connectProcessing(DXL_SIZE);
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
  String *cmd = manipulator.parseDataFromProcessing(data);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      manipulator.actuatorEnable();
      manipulator.sendAngleToProcessing(manipulator.receiveAllActuatorAngle(CHAIN));  
      manipulator.sendToolData2Processing(manipulator.getComponentToolValue(CHAIN, TOOL));
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      manipulator.actuatorDisable();
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

    manipulator.jointMove(CHAIN, goal_position, 1.0f); // FIX TIME PARAM
  }
  else if (cmd[0] == "gripper")
  {
    manipulator.toolMove(CHAIN, TOOL, cmd[1].toFloat());
  }
  else if (cmd[0] == "grip")
  {
    if (cmd[1] == "on")
      manipulator.toolMove(CHAIN, TOOL, 0.0f);
    else if (cmd[1] == "off")
      manipulator.toolMove(CHAIN, TOOL, -1.0f);
  }
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "back")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(-0.010f, 0.0, 0.0), 0.2f);
    else if (cmd[1] == "left")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.010f, 0.0), 0.2f);
    else if (cmd[1] == "right")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, -0.010f, 0.0), 0.2f);
    else if (cmd[1] == "up")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.010f), 0.2f);
    else if (cmd[1] == "down")
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.010f), 0.2f);
    else
      manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.0), 0.2f);
  }
  else if (cmd[0] == "torque")
  {
#ifdef PLATFORM
    if (cmd[1] == "on")
      manipulator.actuatorEnable();
    else if (cmd[1] == "off")
      manipulator.actuatorDisable();
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
  // else if (cmd[0] == "motion")
  // {
  //   if (cmd[1] == "start")
  //   {
  //     if (DYNAMIXEL)
  //       sendAngle2Processing(getAngle()); 

  //     if (PROCESSING)
  //       sendAngle2Processing(getState()); 

  //     for (int i = 0; i < MOTION_NUM; i++)
  //     {
  //       for (int j = 0; j < 6; j++)
  //       {
  //         motion_storage[i][j] = motion_set[i][j];
  //       }
  //     }

  //     motion_num = MOTION_NUM;  
  //     motion_cnt = 0;          
  //     motion = true;
  //     repeat = true;
  //   }
  //   else if (cmd[1] == "stop")
  //   {
  //     motion_cnt = 0;
  //     motion     = false;
  //     repeat     = false;
  //   }
  // }
}

#endif