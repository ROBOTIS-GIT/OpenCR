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

#ifndef LINEAR_H_
#define LINEAR_H_

#include "linear_custom_trajectory.h"
#include "linear_dynamixel.h"
#include "linear_kinematics.h"

#define CUSTOM_TRAJECTORY_SIZE 4
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"

#define DXL_SIZE 4
#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL "tool_dxl"

#define RECEIVE_RATE 0.100 // unit: s
#define CONTROL_RATE 0.010 // unit: s

#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)

class Linear : public robotis_manipulator::RobotisManipulator
{
private:
  robotis_manipulator::Kinematics *kinematics_;
  robotis_manipulator::JointActuator *joint_;  
  robotis_manipulator::ToolActuator *tool_;    
  robotis_manipulator::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

  bool using_actual_robot_state_;
  bool receive_data_flag_ = false;
  double prev_receive_time_ = 0.0;
  double prev_control_time_ = 0.0;

public:
  Linear();
  virtual ~Linear();

  void initDebug();
  void initOpenManipulator(bool using_actual_robot_state, 
                           STRING usb_port = "/dev/ttyUSB0", 
                           STRING baud_rate = "1000000", 
                           float control_rate = CONTROL_RATE);
  void processOpenManipulator(double present_time);

  bool getUsingActualRobotState();
  bool getReceiveDataFlag();
  double getPrevReceiveTime();

  void setReceiveDataFlag(bool receive_data_flag);
  void setPrevReceiveTime(double prev_receive_time);
};

#endif // LINEAR_H_




