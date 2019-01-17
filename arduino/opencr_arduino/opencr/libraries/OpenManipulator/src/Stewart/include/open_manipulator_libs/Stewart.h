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

#ifndef STEWART_H_
#define STEWART_H_

#include "Stewart_Drawing.h"
#include "Stewart_Joint.h"
#include "Stewart_Kinematics.h"
#include "Stewart_Tool.h"

#define NUM_OF_JOINT 6
#define DXL_SIZE 6

#define DRAWING_LINE "drawing_line"
#define DRAWING_CIRCLE "drawing_circle"
#define DRAWING_RHOMBUS "drawing_rhombus"
#define DRAWING_HEART "drawing_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL "tool_dxl"

#define RECEIVE_RATE 0.100 //s
#define CONTROL_RATE 0.010 //s

/* Define X,Y,Z axis */
#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)

class Stewart : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *joint_;  
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;    

  STEWART_DRAWING::Line line_;
  STEWART_DRAWING::Circle circle_;
  STEWART_DRAWING::Rhombus rhombus_;
  STEWART_DRAWING::Heart heart_;

  bool hardware_flag_;
  bool receive_data_flag_ = false;
  bool run_demo_flag_ = false;
  double prev_receive_time_ = 0.0;
  double prev_control_time_ = 0.0;    // not intuitive naming.. control -> execute ? run ?
  int offset_position_num_ = 0;
  std::vector<uint8_t> jointDxlId;

 public:
  Stewart();
  virtual ~Stewart();

  void initDebug();
  void initManipulator(bool hardware_flag,
                       STRING usb_port = "/dev/ttyUSB0", 
                       STRING baud_rate = "1000000");
  void executeData();
  bool getHardwareFlag();
  bool getReceiveDataFlag();
  bool getRunDemoFlag();
  double getPrevReceiveTime();
  int getOffsetPositionNum();

  void setReceiveDataFlag(bool receive_data_flag);
  void setRunDemoFlag(bool run_demo_flag);
  void setPrevReceiveTime(double prev_receive_time);
  void setOffsetPositionNum(int offset_position_num);
};

#endif // STEWART_H_




