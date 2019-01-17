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

#ifndef PROCESSING_H_
#define PROCESSING_H_

#include <Delta.h>

//---------------------------------------------------------------------------------------------------- 
/* Initialize baudrate for Processing */
void initProcessing()
{ 
  Serial.begin(57600);
}

//---------------------------------------------------------------------------------------------------- 
/* Split data by separator */
void split(String data, char separator, String* temp)
{
  int cnt = 0;
  int get_index = 0;

  String copy = data;
  
  while(true)
  {
    get_index = copy.indexOf(separator);

    if(-1 != get_index)
    {
      temp[cnt] = copy.substring(0, get_index);
        copy = copy.substring(get_index + 1);
    }
    else
    {
        temp[cnt] = copy.substring(0, copy.length());
      break;
    }
	  ++cnt;
  }
}

/* Parse data received from Processing */
String* parseProcessingData(String get) 
{
  String cmd[50];
  get.trim();
  split(get, ',', cmd);
  
  return cmd;
}

/* Send joint data(values) to Processing */
void sendJointDataToProcessing(std::vector<WayPoint> joint_angle_vector) 
{
  Serial.print("angle");

  for (int i = 0; i < (int)joint_angle_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(joint_angle_vector.at(i).value,3);
  }
  Serial.print("\n");
}

/* Send tool data(on or off) to Processing */
void sendToolDataToProcessing(bool onoff)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(onoff);
  Serial.print("\n");
}

/* Send tool data(values) to Processing */
void sendToolDataToProcessing(double value)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(value*10);
  Serial.print("\n");
}

/* Send data(values) to Processing */
void sendDataToProcessing(Delta *delta)
{
  // sendJointDataToProcessing(delta->getAllActiveJointValue());
  // sendToolDataToProcessing(delta->getToolValue("tool"));
}

//---------------------------------------------------------------------------------------------------- 
/* Receive data from Processing */
void receiveDataFromProcessing(Delta *delta) 
{
  if (!delta->getReceiveDataFlag())
  {
    if (Serial.available())
    {
      // Read data received from Processing
      String serialInput = Serial.readStringUntil('\n');
      String *cmd = parseProcessingData(serialInput);

      // ...
      // Actuator control tab 
      if (cmd[0] == "actuator")
      {
        if (cmd[1] == "on")
        {
          if(delta->getHardwareFlag())
          {
            delta->allActuatorEnable();
            sendDataToProcessing(delta);  // how..?? can this be used here..??
          }
        }
        else if (cmd[1] == "off")
        {
          if (delta->getHardwareFlag())
          {
            delta->allActuatorDisable();
          }
        }
      }

      // Joint space control tab (not included in Processing)
      else if (cmd[0] == "joint")
      {
        // Torque On/Off
        if (cmd[1] == "on")
        {
          if (delta->getHardwareFlag())    
          {
            delta->allJointActuatorEnable();
            sendJointDataToProcessing(delta->getAllActiveJointValue());
          }
        }
        else if (cmd[1] == "off")
        {
          if (delta->getHardwareFlag())    
            delta->allJointActuatorDisable();
        }

        //
        else
        {
          std::vector<double> goal_position;

          for (uint8_t index = 0; index < DXL_SIZE; index++)
          {
            goal_position.push_back((double)cmd[index + 1].toFloat());
          }
          delta->jointTrajectoryMove(goal_position, 1.0); 
        }
      }

      // Tool control tab 
      else if (cmd[0] == "tool")
      {
        //
        if (cmd[1] == "y")
          delta->toolMove("tool", 1.0);
        else if (cmd[1] == "n")
          delta->toolMove("tool", 0.0);

        // Torque On/Off
        else if (cmd[1] == "on")
        {
          if (delta->getHardwareFlag())    
            delta->allToolActuatorEnable();
        }
        else if (cmd[1] == "off")
        {
          if (delta->getHardwareFlag())    
            delta->allToolActuatorDisable();
        }

        //
        else
          delta->toolMove("tool", (double)cmd[1].toFloat());
      }

      // Task space control tab 
      else if (cmd[0] == "task")
      {
        if (cmd[1] == "f")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.020, 0.0, 0.0), 0.1);
        else if (cmd[1] == "b")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(-0.020, 0.0, 0.0), 0.1);
        else if (cmd[1] == "l")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.020, 0.0), 0.1);
        else if (cmd[1] == "r")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, -0.020, 0.0), 0.1);
        else if (cmd[1] == "u")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, 0.015), 0.1);
        else if (cmd[1] == "d")
          delta->taskTrajectoryMove("tool", RM_MATH::makeVector3(0.0, 0.0, -0.015), 0.1);
        }
      }

      // Demo Control tab
      else if (cmd[0] == "demo")
      {
        if (cmd[1] == "start")
        {
          Pose present_pose = delta->getManipulator()->getComponentPoseFromWorld("tool");
          WayPoint draw_goal_pose[6];
          draw_goal_pose[0].value = present_pose.position(0) + 0.02;
          draw_goal_pose[1].value = present_pose.position(1) + 0.02;
          draw_goal_pose[2].value = present_pose.position(2) - 0.02;
          draw_goal_pose[3].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[0];
          draw_goal_pose[4].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[1];
          draw_goal_pose[5].value = RM_MATH::convertRotationToRPY(present_pose.orientation)[2];

          void *p_draw_goal_pose = &draw_goal_pose;
          delta->drawingTrajectoryMove(DRAWING_LINE, "tool", p_draw_goal_pose, 1.0);
        }
        else if (cmd[1] == "stop")
        {
          double draw_circle_arg[3];
          draw_circle_arg[0] = 0.03; // radius (m)
          draw_circle_arg[1] = 2;    // revolution
          draw_circle_arg[2] = 0.0;  // start angle position (rad)
          void *p_draw_circle_arg = &draw_circle_arg;
          delta->drawingTrajectoryMove(DRAWING_CIRCLE, "tool", p_draw_circle_arg, 4.0);
        }
      }

      // Motion Control tab 
      else if (cmd[0] == "offset")
      {        
        if (cmd[1] == "0") delta->setOffsetPositionNum(0);
        else if (cmd[1] == "1") delta->setOffsetPositionNum(1);
        else if (cmd[1] == "2") delta->setOffsetPositionNum(2);
        else if (cmd[1] == "3") delta->setOffsetPositionNum(3);
        else if (cmd[1] == "4") delta->setOffsetPositionNum(4);
        else if (cmd[1] == "5") delta->setOffsetPositionNum(5);
        else if (cmd[1] == "6") delta->setOffsetPositionNum(6);
      }

      // ...
      delta->setReceiveDataFlag(true);  
      delta->setPrevReceiveTime(millis()/1000.0); // instead of curr_time...?
    }
  }
  else 
  {
    // Serial.println(".");
    // Check if running demo now..
    if (linear->getRunDemoFlag())
    {
      runDemo(linear); 
    }

    // Check if ???
    else if (millis()/1000.0 - linear->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      linear->setReceiveDataFlag(false);   //received <--
      initRC100();
    }
  }
}

#endif  //PROCESSING_H