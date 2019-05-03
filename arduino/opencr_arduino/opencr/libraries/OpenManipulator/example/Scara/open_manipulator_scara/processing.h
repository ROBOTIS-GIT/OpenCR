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

#include <scara_libs.h>
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
void sendToolDataToProcessing(JointWaypoint tool_angle_vector)
{
  Serial.print("tool");
  
  for (int i = 0; i < (int)tool_angle_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(tool_angle_vector.at(i).position,3);
  }
  Serial.print("\n");
}

// Send joint and tool data(values) to Processing 
void sendDataToProcessing(Scara *scara)
{
  sendJointDataToProcessing(scara->getTrajectory()->getManipulator()->getAllJointValue());
  sendToolDataToProcessing(scara->getTrajectory()->getManipulator()->getAllToolValue());
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
void receiveDataFromProcessing(Scara *scara) 
{
  if (!scara->getReceiveDataFlag())
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
          if(scara->getUsingActualRobotState())
          {
            scara->enableAllActuator();
            sendDataToProcessing(scara);
          }
        }
        else if (cmd[1] == "off")
        {
          if(scara->getUsingActualRobotState())
          {
            scara->disableAllActuator();
          }
        }
      }

      // Joint space control tab 
      else if (cmd[0] == "joint")
      {
        // Joint Torque on/off
        if (cmd[1] == "on")
        {
          if (scara->getUsingActualRobotState())    
          {
            scara->enableAllJointActuator();
            sendJointDataToProcessing(scara->getAllActiveJointValue());
          }
        }
        else if (cmd[1] == "off")
        {
          if (scara->getUsingActualRobotState())    
            scara->disableAllJointActuator();
        }

        // Joint Position Control
        else
        {
          std::vector<double> goal_position;
          for (uint8_t index = 0; index < DXL_SIZE; index++)
          {
            goal_position.push_back((double)cmd[index + 1].toFloat());
          }
          scara->makeJointTrajectory(goal_position, 1.0); 
        }
      }

      // Tool control tab 
      else if (cmd[0] == "tool")
      {
        // Tool Torque on/off
        if (cmd[1] == "y")
          scara->makeToolTrajectory("tool", 0.0);
        else if (cmd[1] == "n")
          scara->makeToolTrajectory("tool", -0.5);

        // Torque On/Off
        else if (cmd[1] == "on")
        {
          if (scara->getUsingActualRobotState())    
            scara->enableAllToolActuator();
        }
        else if (cmd[1] == "off")
        {
          if (scara->getUsingActualRobotState())    
            scara->disableAllToolActuator();
        }

        // Tool Position Control
        else
          scara->makeToolTrajectory("tool", (double)cmd[1].toFloat());
      }

      // Task space control tab 
      else if (cmd[0] == "task")
      {
        if (cmd[1] == "f")
          scara->makeTaskTrajectoryFromPresentPose("tool", math::vector3( 0.010, 0.0, 0.0), 1.0);
        else if (cmd[1] == "b")
          scara->makeTaskTrajectoryFromPresentPose("tool", math::vector3(-0.010, 0.0, 0.0), 1.0);
        else if (cmd[1] == "l")
          scara->makeTaskTrajectoryFromPresentPose("tool", math::vector3(0.0,  0.010, 0.0), 1.0);
        else if (cmd[1] == "r")
          scara->makeTaskTrajectoryFromPresentPose("tool", math::vector3(0.0, -0.010, 0.0), 1.0);
        else
          scara->makeTaskTrajectoryFromPresentPose("tool", math::vector3(0.0, 0.0, 0.0), 1.0);
      }

      // Demo Control tab 
      else if (cmd[0] == "demo")
      {
        if (cmd[1] == "start")
          startDemo();
        else if (cmd[1] == "stop")
          stopDemo(scara);
      }

//----------------------------------------------//
//         DO NOT MODIFY THE BELOW CODE         //
//----------------------------------------------//
      scara->setReceiveDataFlag(true);
      scara->setPrevReceiveTime(millis()/1000.0); 
    }
  }
  else 
  {
    if (millis()/1000.0 - scara->getPrevReceiveTime() >= RECEIVE_RATE)
    {
      scara->setReceiveDataFlag(false); 
      initRC100();
    }
  }
}

#endif //PROCESSING_H

