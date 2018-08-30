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

/* Authors: Hye-Jong KIM */

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include "OMLink.h"
#include "Suction.h"
#include "Motion.h"

void connectProcessing()
{
  omlink.connectProcessing(12);
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
  String *cmd = omlink.parseDataFromProcessing(data);

////////////////////////Manipulator OnOff//////////////////////////
  if (cmd[0] == "om")
  {
#ifdef DEBUGFLUG
    DEBUG.print("om-");
#endif
    if (cmd[1] == "ready")
    {
#ifdef PLATFORM
      omlink.actuatorEnable();
      omlink.sendAngleToProcessing(omlink.getAllActiveJointAngle());  
#endif
#ifdef DEBUGFLUG
    DEBUG.print("ready");
#endif
    }
    else if (cmd[1] == "end")
    {
#ifdef PLATFORM
      omlink.actuatorDisable();  
#endif
#ifdef DEBUGFLUG
    DEBUG.print("end");
#endif
    }
#ifdef DEBUGFLUG
    DEBUG.println();
#endif
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Joint Move/////////////////////////////
  else if (cmd[0] == "joint")
  {
#ifdef DEBUGFLUG
    DEBUG.print("joint-");
#endif
    std::vector<float> target_angle;

    for (uint8_t i = 0; i < 3; i++)
    {
      target_angle.push_back(cmd[i+1].toFloat());
    }

#ifdef DEBUGFLUG
    for (uint8_t i = 0; i < 3; i++)
    {
      DEBUG.print(target_angle.at(i));
      DEBUG.print(" , ");
    }
#endif
    omlink.jointMove(target_angle, MOVETIME);
    target_angle.clear();
#ifdef DEBUGFLUG
    DEBUG.println(" ");
#endif
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Suction OnOff///////////////////////////
  else if (cmd[0] == "suction")
  {
#ifdef DEBUGFLUG
    DEBUG.print("suction-");
#endif
    if (cmd[1] == "on")
    {
      suctionOn();
#ifdef DEBUGFLUG
      DEBUG.print("on");
      DEBUG.println();
#endif
    }
    else if (cmd[1] == "off")
    {
      suctionOff();
#ifdef DEBUGFLUG
      DEBUG.print("off");
      DEBUG.println();
#endif
    }
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Task move//////////////////////////////
  else if (cmd[0] == "task")
  {
#ifdef DEBUGFLUG
    DEBUG.print("task-");
#endif
    Pose target_pose;
    std::vector<float> target_angle;

    if (cmd[1] == "forward")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(MOVESTEP, 0.0, 0.0), MOVETIME);
    }  
    else if (cmd[1] == "back")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(-MOVESTEP, 0.0, 0.0), MOVETIME);
    }
    else if (cmd[1] == "left")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(0.0, MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "right")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(0.0, -MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "up")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, MOVESTEP), MOVETIME);
    }
    else if (cmd[1] == "down")
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, -MOVESTEP), MOVETIME);
    }
    else
    {
      omlink.setMove(SUCTION, OM_MATH::makeVector3(0.0, 0.0, 0.0), MOVETIME);
    }
#ifdef DEBUGFLUG
    DEBUG.print(cmd[1]);
    DEBUG.println(" ");
#endif
#ifdef DEBUGFLUG
    DEBUG.print(cmd[1]);
    DEBUG.println(" ");
#endif
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Motor onOff/////////////////////////////
  else if (cmd[0] == "motor")
  {
#ifdef DEBUGFLUG
    DEBUG.print("motor-");
#endif
#ifdef PLATFORM
    if (cmd[1] == "enable")
      {
        omlink.actuatorEnable();
#ifdef DEBUGFLUG
        DEBUG.print("enable");
#endif
      }
      else if (cmd[1] == "disable")
      {
        omlink.actuatorDisable();
#ifdef DEBUGFLUG
        DEBUG.print("disable");
#endif
      }
#endif
    DEBUG.println(" ");
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////teaching///////////////////////////////
  // else if (cmd[0] == "get")
  // {
  //   DEBUG.print("get-");
  //   if (cmd[1] == "on")
  //   {
  //     DEBUG.print("suction on");
  //     DEBUG.println(" ");
  //     motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
  //     motion_storage[filled_motion_num][4] = -1.0;
  //     for (uint8_t j = 1; j < 4; j++)
  //     {
  //       motion_storage[filled_motion_num][j] = 0.0;
  //     }
  //     filled_motion_num++;
  //   }
  //   else if (cmd[1] == "off")
  //   {
  //     DEBUG.print("suction off");
  //     DEBUG.println(" ");
  //     motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
  //     motion_storage[filled_motion_num][4] = 1.0;
  //     for (uint8_t j = 1; j < 4; j++)
  //     {
  //       motion_storage[filled_motion_num][j] = 0.0;
  //     }
  //     filled_motion_num++;
  //   }
  //   else if (cmd[1] == "clear")
  //   {
  //     DEBUG.print("clear");
  //     DEBUG.println(" ");
  //     for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
  //     {
  //       for (uint8_t j = 0; j < 5; j++)
  //       {
  //         motion_storage[i][j] = 0.0;
  //       }
  //     }
      
  //     filled_motion_num = 0;
  //     motion_cnt = 0;
  //     motion     = false;
  //     repeat     = false;
  //   }
  //   else if (cmd[1] == "pose")
  //   {
  //     DEBUG.print("pose");
  //     DEBUG.println(" ");
  //     if (cmd[2].toInt() < MAX_MOTION_NUM)
  //     {
  //       std::vector<float> target_angle = manipulator.getAllActiveJointAngle(OMLINK);

  //       motion_storage[filled_motion_num][0]   = MOVE_TIME;  //mov_time
  //       for (uint8_t i = 0; i < 3; i++)
  //       {
  //         motion_storage[filled_motion_num][i+1] = target_angle.at(i);
  //       }
  //       motion_storage[filled_motion_num][4] = 0.0;
  //       filled_motion_num++;
  //     }
  //   }
  // }
////////////////////////////////////////////////////////////////////
//////////////////////////teaching move/////////////////////////////
  // else if (cmd[0] == "hand")
  // {
  //   DEBUG.print("hand-");
  //   if (cmd[1] == "once")
  //   {
  //     DEBUG.print("once");
  //     DEBUG.println(" ");
  //     if (DYNAMIXEL)
  //     {
  //       manipulator.actuatorEnable();
  //       //PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK)); 
  //     }

  //     motion_cnt = 0;
  //     motion = true;
  //     repeat = false;
  //   }
  //   else if (cmd[1] == "repeat")
  //   {
  //     DEBUG.print("repeat");
  //     DEBUG.println(" ");
  //     if (DYNAMIXEL)
  //     {
  //       manipulator.actuatorEnable();
  //       //PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK)); 
  //     }

  //     motion_cnt = 0;
  //     motion = true;
  //     repeat = true;
  //   }
  //   else if (cmd[1] == "stop")
  //   {
  //     DEBUG.print("stop");
  //     DEBUG.println(" ");
  //     for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
  //     {
  //       for (uint8_t j = 0; j < 5; j++)
  //       {
  //         motion_storage[i][j] = 0.0;
  //       }
  //     }
  //     motion_cnt = 0;
  //     motion     = false;
  //     repeat     = false;
  //   }
  // }
////////////////////////////////////////////////////////////////////
///////////////////////////motion move//////////////////////////////
  else if (cmd[0] == "motion")
  {
#ifdef DEBUGFLUG
    DEBUG.print("motion-");
#endif  
    if (cmd[1] == "start")
    {
#ifdef DEBUGFLUG
      DEBUG.print("start");
#endif
      motionStart();
    }
    else if (cmd[1] == "stop")
    {
#ifdef DEBUGFLUG
      DEBUG.print("stop");
#endif  
      motionStop();
    }
#ifdef DEBUGFLUG
    DEBUG.println(" ");
#endif   
  }
}
////////////////////////////////////////////////////////////////////
#endif