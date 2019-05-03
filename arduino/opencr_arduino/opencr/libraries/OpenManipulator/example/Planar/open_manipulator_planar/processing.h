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

#include <planar_libs.h>
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
** Send data to Processing
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

// Send joint and tool values to Processing 
void sendDataToProcessing(Planar *planar)
{
  sendJointDataToProcessing(planar->getTrajectory()->getManipulator()->getAllJointValue());
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
void receiveDataFromProcessing(Planar *planar) 
{
  if (!planar->getReceiveDataFlag())
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
          if(planar->getUsingActualRobotState())
          {
            planar->enableAllActuator();
            sendDataToProcessing(planar);
          }
        }
        else if (cmd[1] == "off")
        {
          if(planar->getUsingActualRobotState())
          {
            planar->disableAllActuator();
          }
        }
      }

      // Joint space control tab 
      else if (cmd[0] == "joint")
      {
        // Joint Torque on/off
        if (cmd[1] == "on")
        {
          if (planar->getUsingActualRobotState())    
          {
            planar->enableAllJointActuator();
            sendJointDataToProcessing(planar->getTrajectory()->getManipulator()->getAllJointValue());
          }
        }
        else if (cmd[1] == "off")
        {
          if (planar->getUsingActualRobotState())    
            planar->disableAllJointActuator();
        }

        // Joint Position Control
        // else
        // {
        //   std::vector<double> goal_position;
        //   for (uint8_t index = 0; index < DXL_SIZE; index++)
        //   {
        //     goal_position.push_back((double)cmd[index + 1].toFloat());
        //   }
        //   planar->makeJointTrajectory(goal_position, 1.0); 
        // }
      }

      // Task space control tab 
      else if (cmd[0] == "task")
      {
        if (cmd[1] == "f")
          planar->makeTaskTrajectory("tool", math::vector3( 0.020, 0.0, 0.0), 0.15);
        else if (cmd[1] == "b")
          planar->makeTaskTrajectory("tool", math::vector3(-0.020, 0.0, 0.0), 0.15);
        else if (cmd[1] == "l")
          planar->makeTaskTrajectory("tool", math::vector3(0.0,  0.020, 0.0), 0.15);
        else if (cmd[1] == "r")
          planar->makeTaskTrajectory("tool", math::vector3(0.0, -0.020, 0.0), 0.15);
        else
          planar->makeTaskTrajectory("tool", math::vector3(0.0, 0.0, 0.0), 0.15);
      }

      // Demo Control tab
      else if (cmd[0] == "demo")
      {
        if (cmd[1] == "start")
          startDemo();
        else if (cmd[1] == "stop")
          stopDemo(planar);
      }

//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      planar->setReceiveDataFlag(true);
      planar->setPrevReceiveTime(millis()/1000.0); 
    }
  }
  else 
  {
    if (millis()/1000.0 - planar->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      planar->setReceiveDataFlag(false); 
      initRC100();
    }
  }
}

#endif //PROCESSING_H
