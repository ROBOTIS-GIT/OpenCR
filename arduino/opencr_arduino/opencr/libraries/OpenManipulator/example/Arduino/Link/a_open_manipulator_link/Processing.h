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
  
namespace MyProcessing
{
  Pose setPose(String dir)
  {
    Pose target_pose;
    float step = 0.02;

    target_pose = manipulator.getComponentPoseToWorld(OMLINK, SUCTION);

    if (dir == "forward")
    {
      target_pose.position(0) += step;
    }  
    else if (dir == "back")
    {
      target_pose.position(0) -= step;
    }
    else if (dir == "left")
    {
      target_pose.position(1) += step;
    }
    else if (dir == "right")
    {
      target_pose.position(1) -= step;
    }
    else if (dir == "up")
    {
      target_pose.position(2) += step;
    }
    else if (dir == "down")
    {
      target_pose.position(2) -= step;
    }

    DEBUG.print(target_pose.position(0));
    DEBUG.print(" , ");
    DEBUG.print(target_pose.position(1));
    DEBUG.print(" , ");
    DEBUG.print(target_pose.position(2));
    DEBUG.println(" ");

    return target_pose;
  }

  void dataFromProcessing(String get)
  {
    get.trim();
    OM_PROCESSING::split(get, ',', cmd);
    DEBUG.print(cmd[0]);
    DEBUG.print(",");
    DEBUG.print(cmd[1]);
    DEBUG.print(" / ");
    DEBUG.print("do : ");

    if (cmd[0] == "om")
    {
      DEBUG.print("om-");
      if (cmd[1] == "ready")
      {
        DEBUG.print("ready");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          manipulator.actuatorEnable();
        }

        if (PROCESSINGON)
          send_processing_flug = true ;
          //PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK)); 
      }
      else if (cmd[1] == "end")
      {
        DEBUG.print("end");
        DEBUG.println(" ");
        send_processing_flug = false ;
        if (DYNAMIXEL)
          manipulator.actuatorDisable();
      }
    }
    else if (cmd[0] == "joint")
    {
      DEBUG.print("joint-");
      std::vector<float> target_angle;

      for (uint8_t i = 1; i < 4; i++)
        target_angle.push_back(cmd[i].toFloat());
      MyFunction::updateJointTrajectory(target_angle, MOVE_TIME);
      DEBUG.print(target_angle.at(0));
      DEBUG.print(" , ");
      DEBUG.print(target_angle.at(1));
      DEBUG.print(" , ");
      DEBUG.print(target_angle.at(2));
      DEBUG.println(" ");
    }
    else if (cmd[0] == "suction")
    {
      DEBUG.print("suction-");
      if (cmd[1] == "on")
      {
        DEBUG.print("on");
        DEBUG.println(" ");
      }//suction on
        
      else if (cmd[1] == "off")
      {
        DEBUG.print("off");
        DEBUG.println(" ");
      }//suction off
        
      else{}
    }
    else if (cmd[0] == "task")
    {
      DEBUG.print("task-");
      Pose target_pose;
      std::vector<float> target_angle;

      target_pose = setPose(cmd[1]);
      target_angle = manipulator.inverse(OMLINK, SUCTION, target_pose);

      MyFunction::updateJointTrajectory(target_angle, MOVE_TIME);
      DEBUG.println(" ");
    }
    else if (cmd[0] == "motor")
    {
      DEBUG.print("motor-");
      if (DYNAMIXEL)
      {
        if (cmd[1] == "enable")
        {
          DEBUG.print("enable");
          DEBUG.println(" ");
          manipulator.actuatorEnable();
        }
        else if (cmd[1] == "disable")
        {
          DEBUG.print("disable");
          DEBUG.println(" ");
          manipulator.actuatorDisable();
        }
      }
    }
    else if (cmd[0] == "get")
    {
      DEBUG.print("get-");
      if (cmd[1] == "on")
      {
        DEBUG.print("suction on");
        DEBUG.println(" ");
        motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
        motion_storage[filled_motion_num][4] = -1.0;
        for (uint8_t j = 1; j < 4; j++)
        {
          motion_storage[filled_motion_num][j] = 0.0;
        }
        filled_motion_num++;
      }
      else if (cmd[1] == "off")
      {
        DEBUG.print("suction off");
        DEBUG.println(" ");
        motion_storage[filled_motion_num][0] = MOVE_TIME;  //mov_time
        motion_storage[filled_motion_num][4] = 1.0;
        for (uint8_t j = 1; j < 4; j++)
        {
          motion_storage[filled_motion_num][j] = 0.0;
        }
        filled_motion_num++;
      }
      else if (cmd[1] == "clear")
      {
        DEBUG.print("clear");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        
        filled_motion_num = 0;
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
      else if (cmd[1] == "pose")
      {
        DEBUG.print("pose");
        DEBUG.println(" ");
        if (cmd[2].toInt() < MAX_MOTION_NUM)
        {
          std::vector<float> target_angle = manipulator.getAllActiveJointAngle(OMLINK);

          motion_storage[filled_motion_num][0]   = MOVE_TIME;  //mov_time
          for (uint8_t i = 0; i < 3; i++)
          {
            motion_storage[filled_motion_num][i+1] = target_angle.at(i);
          }
          motion_storage[filled_motion_num][4] = 0.0;
          filled_motion_num++;
        }
      }
    }
    else if (cmd[0] == "hand")
    {
      DEBUG.print("hand-");
      if (cmd[1] == "once")
      {
        DEBUG.print("once");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          manipulator.actuatorEnable();
          //PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK)); 
        }

        motion_cnt = 0;
        motion = true;
        repeat = false;
      }
      else if (cmd[1] == "repeat")
      {
        DEBUG.print("repeat");
        DEBUG.println(" ");
        if (DYNAMIXEL)
        {
          manipulator.actuatorEnable();
          //PROCESSING::sendAngle2Processing(manipulator.getAllJointAngle(OMLINK)); 
        }

        motion_cnt = 0;
        motion = true;
        repeat = true;
      }
      else if (cmd[1] == "stop")
      {
        DEBUG.print("stop");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
    }
    else if (cmd[0] == "motion")
    {
      DEBUG.print("motion-");
      if (cmd[1] == "start")
      {
        DEBUG.print("start");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = initial_motion_set[i][j];
          }
        }

        filled_motion_num = MAX_MOTION_NUM;  
        motion_cnt = 0;          
        motion = true;
        repeat = true;
      }
      else if (cmd[1] == "stop")
      {
        DEBUG.print("stop");
        DEBUG.println(" ");
        for (uint8_t i = 0; i < MAX_MOTION_NUM; i++)
        {
          for (uint8_t j = 0; j < 5; j++)
          {
            motion_storage[i][j] = 0.0;
          }
        }
        motion_cnt = 0;
        motion     = false;
        repeat     = false;
      }
    }
  }

  void getData(uint32_t wait_time)
  {
    static uint8_t state = 0;
    static uint32_t tick = 0;

    static bool processing_flag = false;
    String get_processing_data = "";

    if (Serial.available())
    {
      get_processing_data = Serial.readStringUntil('\n');
      processing_flag = true;
    }

    if(processing_flag)
    {
      switch (state)
      {
        case CHECK_FLAG:
            DEBUG.print("data : ");
            dataFromProcessing(get_processing_data);   
            tick = millis();
            state = WAIT_FOR_SEC;
        break;
        
        case WAIT_FOR_SEC:
          if ((millis() - tick) >= wait_time)
          {
            processing_flag = false;
            state = CHECK_FLAG;
          }
        break;
        
        default :
        state = CHECK_FLAG;
        break;
      }
    }
  }

}// namespace MyProcessing
#endif