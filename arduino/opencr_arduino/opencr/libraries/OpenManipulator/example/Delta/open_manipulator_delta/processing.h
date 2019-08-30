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

#include <delta_libs.h>
#include "demo.h"

/*****************************************************************************
** Initialize baudrate for using Processing
*****************************************************************************/
void initProcessing()
{ 
  Serial.begin(57600);
  
  Serial.print("Initial actuator angles: ");
  for (int i = 0; i < DXL_SIZE; i++)
  {
    Serial.print(0.0);
    Serial.print(",");
  }
  Serial.println("");
  delay(300);             
}

/*****************************************************************************
** Send data from Processing
*****************************************************************************/
// Send joint data to Processing 
void sendJointDataToProcessing(JointWaypoint joint_angle_vector) 
{
  Serial.print("joint");

  for (int i = 0; i < (int)joint_angle_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(joint_angle_vector.at(i).position,3);
  }
  Serial.print("\n");
}

// Send tool data(on or off) to Processing 
void sendToolDataToProcessing(bool onoff)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(onoff);
  Serial.print("\n");
}

// Send tool data(values) to Processing 
void sendToolDataToProcessing(JointValue value)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(value.position);
  Serial.print("\n");
}

// Send joint and tool data(values) to Processing 
void sendDataToProcessing(Delta *delta)
{
  sendJointDataToProcessing(delta->getTrajectory()->getManipulator()->getAllJointValue());
}

/*****************************************************************************
** Receive data from Processing
*****************************************************************************/
// Split data by separator 
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

// Parse data received from Processing 
String cmd[50];
String* parseProcessingData(String get) 
{
  get.trim();
  split(get, ',', cmd);
  
  return cmd;
}

// Receive data from Processing 
void receiveDataFromProcessing(Delta *delta) 
{
  if (!delta->getReceiveDataFlag())
  {
    if (Serial.available())
    {
      String serialInput = Serial.readStringUntil('\n');
      String *cmd = parseProcessingData(serialInput);

      // Actuator Control Tab 
      if (cmd[0] == "actuator")
      {
        // Torque On/Off
        if (cmd[1] == "on")
        {
          if(delta->getUsingActualRobotState())
          {
            delta->enableAllActuator();
            sendDataToProcessing(delta);
          }
        }
        else if (cmd[1] == "off")
        {
          if(delta->getUsingActualRobotState())
          {
            delta->disableAllActuator();
          }
        }
      }

      // Joint space control tab 
      else if (cmd[0] == "joint")
      {
        // Joint Torque on/off
        if (cmd[1] == "on")
        {
          if (delta->getUsingActualRobotState())    
          {
            delta->enableAllJointActuator();
            sendJointDataToProcessing(delta->getAllActiveJointValue());
          }
        }
        else if (cmd[1] == "off")
        {
          if (delta->getUsingActualRobotState())    
            delta->disableAllJointActuator();
        }

        // Joint Position Control
        // else
        // {
        //   std::vector<double> goal_position;
        //   for (uint8_t index = 0; index < DXL_SIZE; index++)
        //   {
        //     goal_position.push_back((double)cmd[index + 1].toFloat());
        //   }
        //   delta->makeJointTrajectory(goal_position, 1.0); 
        // }
      }

      // Tool control tab 
      else if (cmd[0] == "tool")
      {
        if (cmd[1] == "y")
          suctionOnOff(true);
        else if (cmd[1] == "n")
          suctionOnOff(false);
      }

      // Task space control tab 
      else if (cmd[0] == "task")
      {
        // if (cmd[1] == "f")
        //   delta->makeTaskTrajectory("tool", math::vector3(0.020, 0.0, 0.0), 0.1);
        // else if (cmd[1] == "b")
        //   delta->makeTaskTrajectory("tool", math::vector3(-0.020, 0.0, 0.0), 0.1);
        // else if (cmd[1] == "l")
        //   delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.020, 0.0), 0.1);
        // else if (cmd[1] == "r")
        //   delta->makeTaskTrajectory("tool", math::vector3(0.0, -0.020, 0.0), 0.1);
        // else if (cmd[1] == "u")
        //   delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.015), 0.1);
        // else if (cmd[1] == "d")
        //   delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.010), 0.1);
        // else
        //   delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 0.1);
        if (cmd[1] == "f")
          delta->makeTaskTrajectory("tool", math::vector3(0.040, 0.0, 0.02), 0.1);
        else if (cmd[1] == "b")
          delta->makeTaskTrajectory("tool", math::vector3(-0.040, 0.0, 0.02), 0.1);
        else if (cmd[1] == "l")
          delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.040, 0.02), 0.1);
        else if (cmd[1] == "r")
          delta->makeTaskTrajectory("tool", math::vector3(0.0, -0.040, 0.02), 0.1);
        else if (cmd[1] == "u")
          delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.05), 0.1);
        else if (cmd[1] == "d")
          delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, -0.010), 0.1);
        else
          delta->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 0.1);
      }

      // Demo Control tab 
      else if (cmd[0] == "demo")
      {
        if (cmd[1] == "start")
          startDemo();
        else if (cmd[1] == "stop")
          stopDemo(delta);
      }

//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      delta->setReceiveDataFlag(true);
      delta->setPrevReceiveTime(millis()/1000.0); 
    }
  }
  else
  {
    if (millis()/1000.0 - delta->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      delta->setReceiveDataFlag(false); 
      initRC100();
    }
  }
}

#endif  //PROCESSING_H
