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

#include "Link.h"
#include "Motion.h"

//////////////////Move step/////////////////
#define MOVESTEP 0.01
////////////////////////////////////////////
//////////////////Move time/////////////////
#define MOVETIME 1.0
////////////////////////////////////////////
//////////////////cmd param/////////////////
String global_cmd[50];
////////////////////////////////////////////


////////////////////////////send////////////////////////////////////
void sendAngle2Processing(std::vector<double> joint_angle_vector)
{
  Serial.print("angle");

  for (uint32_t i = 0; i < joint_angle_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(joint_angle_vector.at(i));
  }
  Serial.print("\n");
}

void sendToolData2Processing(bool onoff)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(onoff);
  Serial.print("\n");
}

void sendToolData2Processing(double value)
{
  Serial.print("tool");
  Serial.print(",");
  Serial.print(value);
  Serial.print("\n");
}
////////////////////////////////////////////////////////////////////

void connectProcessing()
{
  for (int i = 0; i < 5; i++)
  {
    Serial.print(0.0);
    Serial.print(",");
  }

  Serial.println(0.0);
  delay(300);

  Serial.println("Init Processing");
}

int availableProcessing()
{
  return Serial.available();
}

String readProcessingData()
{
  return Serial.readStringUntil('\n');
}

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

String* parseDataFromProcessing(String get)
{
  get.trim();
  split(get, ',', global_cmd);
  
  return global_cmd;
}

void fromProcessing(OPEN_MANIPULATOR_LINK *omlink, String data)
{
  String *cmd = parseDataFromProcessing(data);

////////////////////////Manipulator OnOff//////////////////////////
  if (cmd[0] == "om")
  {
    if (cmd[1] == "ready")
    {
      if(omlink->getPlatformFlag())
      {
        omlink->allActuatorEnable();
        sendAngle2Processing(omlink->getManipulator()->getAllActiveJointValue());
        sendToolData2Processing(omlink->getManipulator()->getValue("vacuum"));
      }
    }
    else if (cmd[1] == "end")
    {
      if(omlink->getPlatformFlag())
      {
        omlink->allActuatorDisable();
      }
    }
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Joint Move/////////////////////////////
  else if (cmd[0] == "joint")
  {
    std::vector<double> goal_position;

    for (int8_t index = 0; index < omlink->getManipulator()->getDOF(); index++)
    {
      goal_position.push_back((double)cmd[index + 1].toFloat());
    }

    omlink->jointTrajectoryMove(goal_position, MOVETIME); // FIX TIME PARAM
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Vacuum OnOff///////////////////////////
  else if (cmd[0] == "suction")
  {
    if (cmd[1] == "on")
    {
      omlink->toolMove("vacuum", 1.0);
    }
    else if (cmd[1] == "off")
    {
      omlink->toolMove("vacuum", -1.0);
    }
  }
////////////////////////////////////////////////////////////////////
/////////////////////////////Task move//////////////////////////////
  else if (cmd[0] == "task")
  {
    Pose target_pose;
    std::vector<double> target_angle;

    if (cmd[1] == "forward")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(MOVESTEP, 0.0, 0.0), MOVETIME);
    }  
    else if (cmd[1] == "back")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(-MOVESTEP, 0.0, 0.0), MOVETIME);
    }
    else if (cmd[1] == "left")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(0.0, MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "right")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(0.0, -MOVESTEP, 0.0), MOVETIME);
    }
    else if (cmd[1] == "up")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(0.0, 0.0, MOVESTEP), MOVETIME);
    }
    else if (cmd[1] == "down")
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(0.0, 0.0, -MOVESTEP), MOVETIME);
    }
    else
    {
      omlink->taskTrajectoryMoveToPresentPose("vacuum", RM_MATH::makeVector3(0.0, 0.0, 0.0), MOVETIME);
    }
  }
////////////////////////////////////////////////////////////////////
////////////////////////////Motor onOff/////////////////////////////
  else if (cmd[0] == "motor")
  {
    if(omlink->getPlatformFlag())
    {
      if (cmd[1] == "enable")
      {
        omlink->allActuatorEnable();
      }
      else if (cmd[1] == "disable")
      {
        omlink->allActuatorEnable();
      }
    }
  }
////////////////////////////////////////////////////////////////////
///////////////////////////motion move//////////////////////////////
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "start")
    {
      motionStart(omlink);
    }
    else if (cmd[1] == "stop")
    {
      motionStop();
    }
  }
////////////////////////////////////////////////////////////////////
}

#endif