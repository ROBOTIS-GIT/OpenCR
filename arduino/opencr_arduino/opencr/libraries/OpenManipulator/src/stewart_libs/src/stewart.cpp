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

#include "../include/stewart_libs/stewart.h"

Stewart::Stewart() {}
Stewart::~Stewart()
{
  delete kinematics_;
  delete joint_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void Stewart::initDebug()
{
  DEBUG.begin(57600); // Using Serial4(=SerialBT2)
  log::print("OpenManipulator Debugging Port"); 
}

void Stewart::initOpenManipulator(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_rate)
{
  /*****************************************************************************
  ** Set if using actual robot
  *****************************************************************************/
  using_actual_robot_state_ = using_actual_robot_state;  

  /*****************************************************************************
  ** Initialize Manipulator Parameters
  *****************************************************************************/
  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1",  // my name
           "world",   // parent name
           "joint2",  // child name
           math::vector3(0.0, 0.0, 0.400),                  // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           1);        // actuator id

  addJoint("joint2",  // my name
           "joint1",  // parent name
           "joint3",  // child name
           math::vector3(0.0, 0.0, 0.0),                    // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           2);        // actuator id

  addJoint("joint3",  // my name
           "joint2",  // parent name
           "joint4",  // child name
           math::vector3(0.0, 0.0, 0.0),                    // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           3);        // actuator id

  addJoint("joint4",  // my name
           "joint3",  // parent name
           "joint5",  // child name
           math::vector3(0.0, 0.0, 0.0),                    // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           4);        // actuator id

  addJoint("joint5",  // my name
           "joint4",  // parent name
           "joint6",  // child name
           math::vector3(0.0, 0.0, 0.0),                    // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           5);        // actuator id

  addJoint("joint6",  // my name
           "joint5",  // parent name
           "tool",    // child name
           math::vector3(0.0, 0.0, 0.0),                    // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           6);        // actuator id

  addTool("tool",     // my name
          "joint6",   // parent name
          math::vector3(0.0, 0.0, 0.0),                    // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          -1,         // actuator id
          -0.015);    // Change unit from `meter` to `radian` <<--- modify!!!!

  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new stewart_kinematics::SolverUsingGeometry();
  addKinematics(kinematics_);
  
  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new stewart_custom_trajectory::Line();
  custom_trajectory_[1] = new stewart_custom_trajectory::Circle();
  custom_trajectory_[2] = new stewart_custom_trajectory::Rhombus();
  custom_trajectory_[3] = new stewart_custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);

  if (using_actual_robot_state_)
  {
    /*****************************************************************************
    ** Initialize ㅓoint Actuator
    *****************************************************************************/
    joint_ = new stewart_dynamixel::JointDynamixelProfileControl(control_rate);

    // Set communication arguments 
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg; 

    // Set joint actuator id 
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(1);
    jointDxlId.push_back(2);
    jointDxlId.push_back(3);
    jointDxlId.push_back(4);
    jointDxlId.push_back(5);
    jointDxlId.push_back(6);
    addJointActuator(JOINT_DYNAMIXEL, joint_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    /*****************************************************************************
    ** Enable actuators and Receive actuator values 
    *****************************************************************************/
    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
  }
}

/*****************************************************************************
** Process actuator values received from external controllers
*****************************************************************************/
void Stewart::processOpenManipulator(double present_time)  
{
  if (present_time - prev_control_time_ >= CONTROL_RATE)  
  {
    JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);

    if (using_actual_robot_state_)
    {
      receiveAllJointActuatorValue();   
    }

    if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);   

    // Set previous control time
    prev_control_time_ = millis()/1000.0;
  }
}

/*****************************************************************************
** State Functions
*****************************************************************************/
// Check if using acutal robot 
bool Stewart::getUsingActualRobotState() 
{
  return using_actual_robot_state_;
}

/* Check if the program read data within control rate) */
bool Stewart::getReceiveDataFlag()  
{
  return receive_data_flag_;
}

/* Get the previous time when data were received */
double Stewart::getPrevReceiveTime() 
{
  return prev_receive_time_;
}

/* Set whether data were received or not */
void Stewart::setReceiveDataFlag(bool receive_data_flag) 
{
  receive_data_flag_ = receive_data_flag;
}

/* Set the previous time when data were received */
void Stewart::setPrevReceiveTime(double prev_receive_time)  
{
  prev_receive_time_ = prev_receive_time;
}