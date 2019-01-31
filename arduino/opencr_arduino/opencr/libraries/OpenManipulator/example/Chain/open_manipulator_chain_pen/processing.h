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

#include "open_manipulator_pen.h"

#define DXL_SIZE 5

typedef struct _MotionWaypoint
{
  std::vector<double> angle;
  double path_time;
  double gripper_value;
} MotionWaypoint;

std::vector<MotionWaypoint> motion_way_point_buf;
bool processing_motion_state = false;
char hand_motion_cnt = 0;
bool hand_motion_repeat_state = false;
bool platform_state_processing = false;
String global_cmd[50];

void connectProcessing(bool platform)
{ 
  platform_state_processing = platform;
  for (int i = 0; i < DXL_SIZE; i++)
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

void sendAngleToProcessing(JointWaypoint joint_states_vector)
{
  Serial.print("angle");

  for (int i = 0; i < (int)joint_states_vector.size(); i++)
  {
    Serial.print(",");
    Serial.print(joint_states_vector.at(i).position, 3);
  }
  Serial.print("\n");
}

void sendValueToProcessing(OpenManipulatorPen *open_manipulator)
{
  sendAngleToProcessing(open_manipulator->getAllActiveJointValue());
}

void fromProcessing(OpenManipulatorPen *open_manipulator, String data)
{
  String *cmd = parseDataFromProcessing(data);

  if (cmd[0] == "opm")
  {
    if (cmd[1] == "ready")
    {
      if(platform_state_processing)
      {
        open_manipulator->enableAllActuator();
        sendValueToProcessing(open_manipulator);
      }
    }
    else if (cmd[1] == "end")
    {
      if(platform_state_processing)
      {
        open_manipulator->disableAllActuator();
      }
    }
  }
  ////////// joint space control tab
  else if (cmd[0] == "joint")
  {
    std::vector<double> goal_position;
    for (uint8_t index = 0; index < DXL_SIZE; index++)
    {
      goal_position.push_back((double)cmd[index + 1].toFloat());
    }
    open_manipulator->makeJointTrajectory(goal_position, 1.0); // FIX TIME PARAM
  }
  ////////// task space control tab
  else if (cmd[0] == "task")
  {
    if (cmd[1] == "forward")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.010, 0.0, 0.0), 0.2);
    else if (cmd[1] == "back")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(-0.010, 0.0, 0.0), 0.2);
    else if (cmd[1] == "left")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.010, 0.0), 0.2);
    else if (cmd[1] == "right")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, -0.010, 0.0), 0.2);
    else if (cmd[1] == "up")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.010), 0.2);
    else if (cmd[1] == "down")
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, -0.010), 0.2);
    else
      open_manipulator->makeTaskTrajectoryFromPresentPose("pen", math::vector3(0.0, 0.0, 0.0), 0.2);
  }
  else if (cmd[0] == "torque")
  {
    if(platform_state_processing)
    {
      if (cmd[1] == "on")
        open_manipulator->enableAllActuator();
      else if (cmd[1] == "off")
        open_manipulator->disableAllActuator();
    }
  }
  ////////// hand teaching tab
  else if (cmd[0] == "get")
  {
    if (cmd[1] == "clear")  // motion clear
    {
      processing_motion_state = false;
      motion_way_point_buf.clear();
      hand_motion_cnt = 0;
    }
    else if (cmd[1] == "pose")  // save pose
    {
      MotionWaypoint read_value;
      JointWaypoint present_states = open_manipulator->getAllActiveJointValue();
      for(uint32_t i = 0; i < present_states.size(); i ++)
        read_value.angle.push_back(present_states.at(i).position);  
      read_value.path_time = 2.0; // FIX TIME PARAM
      motion_way_point_buf.push_back(read_value);  
      hand_motion_cnt = 0;
    }
  }
  else if (cmd[0] == "hand")
  {
    if (cmd[1] == "once") // play motion (once)
    {
      processing_motion_state = true;//
    }
    else if (cmd[1] == "repeat") // play motion (repeat)
    {
      hand_motion_repeat_state = true;
    }
    else if (cmd[1] == "stop") // play motion (stop)
    {
      hand_motion_repeat_state = false;
      processing_motion_state = false;
      hand_motion_cnt = 0;
    }
  }
  ////////// motion tab
  else if (cmd[0] == "motion")
  {
    if (cmd[1] == "1")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = 0.02;
      draw_line_arg.kinematic.position(1) = 0.02;
      draw_line_arg.kinematic.position(2) = -0.02;
      void *p_draw_line_arg = &draw_line_arg;
      open_manipulator->makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, "pen", p_draw_line_arg, 1.0);
    }
    else if (cmd[1] == "2")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = 0.03; // radius (m)
      draw_circle_arg[1] = 2;    // revolution
      draw_circle_arg[2] = 0.0;  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;
      open_manipulator->makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, "pen", p_draw_circle_arg, 4.0);
    }
  }
}

void playProcessingMotion(OpenManipulatorPen *open_manipulator)
{
  if(!open_manipulator->getMovingState() && processing_motion_state)
  {
    if(motion_way_point_buf.size() == 0)
      return;

    open_manipulator->makeJointTrajectory(motion_way_point_buf.at(hand_motion_cnt).angle, motion_way_point_buf.at(hand_motion_cnt).path_time); 
    hand_motion_cnt ++;
    if(hand_motion_cnt >= motion_way_point_buf.size())
    {
      hand_motion_cnt = 0;
      if(!hand_motion_repeat_state)
        processing_motion_state = false;
    }
  }
}

#endif
