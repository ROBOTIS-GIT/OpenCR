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

#include "../include/linear_libs/linear.h"

Linear::Linear() {}
Linear::~Linear()
{
  delete kinematics_;
  delete joint_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void Linear::initDebug()
{
  DEBUG.begin(57600); // Using Serial4(=SerialBT2)
  log::print("OpenManipulator Debugging Port"); 
}

void Linear::initOpenManipulator(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_rate)
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
           math::vector3(0.0, 0.0, 0.0),                    // Not used
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // Not used
           Z_AXIS,    // axis of rotation
           1,         // actuator id
           5.0/3.0*M_PI,   // max_limit
           -5.0/3.0*M_PI); // min_limit
                 

  addJoint("joint2",  // my name
           "joint1",  // parent name
           "joint3",  // child name
           math::vector3(0.0, 0.0, 0.0),                    // Not used
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // Not used
           Z_AXIS,    // axis of rotation
           2,         // actuator id
           5.0/3.0*M_PI,   // max_limit
           -5.0/3.0*M_PI); // min_limit
                  

  addJoint("joint3",  // my name
           "joint2",  // parent name
           "tool",    // child name
           math::vector3(0.0, 0.0, 0.0),                    // Not used
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // Not used
           Z_AXIS,    // axis of rotation
           3,         // actuator id
           2.1*M_PI,  // max_limit
           -2.1*M_PI);// min_limit

  addTool("tool",     // my name
          "joint3",   // parent name
          math::vector3(0.0, 0.0, 0.0),                    // Not used
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // Not used
          4,          // actuator id
          0.500,      // max gripper limit 
          -0.500);    // min gripper limit 

  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new linear_kinematics::SolverUsingGeometry();
  addKinematics(kinematics_);

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new linear_custom_trajectory::Line();
  custom_trajectory_[1] = new linear_custom_trajectory::Circle();
  custom_trajectory_[2] = new linear_custom_trajectory::Rhombus();
  custom_trajectory_[3] = new linear_custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);

  if (using_actual_robot_state_)
  {
    /*****************************************************************************
    ** Initialize ㅓoint Actuator
    *****************************************************************************/
    joint_ = new linear_dynamixel::JointDynamixelProfileControl(control_rate);

    // Set communication arguments 
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg; 

    // Set joint actuator id 
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(1);
    jointDxlId.push_back(2);
    jointDxlId.push_back(3);
    addJointActuator(JOINT_DYNAMIXEL, joint_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new linear_dynamixel::ToolDynamixel();

    // Set tool actuator id 
    uint8_t toolDxlId = 4;
    addToolActuator(TOOL_DYNAMIXEL, tool_, toolDxlId, p_dxl_comm_arg);

    // Set tool actuator control mode 
    STRING tool_dxl_mode_arg = "position_mode";
    void *p_tool_dxl_mode_arg = &tool_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_tool_dxl_mode_arg);

    /*****************************************************************************
    ** Enable actuators and Receive actuator values 
    *****************************************************************************/
    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }
}

/*****************************************************************************
** Process actuator values received from external controllers
*****************************************************************************/
void Linear::processOpenManipulator(double present_time)  
{
  if (present_time - prev_control_time_ >= CONTROL_RATE)  
  {
    JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
    JointWaypoint goal_tool_value  = getToolGoalValue();

    if (using_actual_robot_state_)
    {
      receiveAllJointActuatorValue();   
      receiveAllToolActuatorValue();    
    }

    if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);   
    if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);
    solveForwardKinematics(); 

    // Set previous control time
    prev_control_time_ = millis()/1000.0;
  }
}

/*****************************************************************************
** State Functions
*****************************************************************************/
// Check if using acutal robot 
bool Linear::getUsingActualRobotState() 
{
  return using_actual_robot_state_;
}

/* Check if the program read data within control rate) */
bool Linear::getReceiveDataFlag()  
{
  return receive_data_flag_;
}

/* Get the previous time when data were received */
double Linear::getPrevReceiveTime() 
{
  return prev_receive_time_;
}

/* Set whether data were received or not */
void Linear::setReceiveDataFlag(bool receive_data_flag) 
{
  receive_data_flag_ = receive_data_flag;
}

/* Set the previous time when data were received */
void Linear::setPrevReceiveTime(double prev_receive_time)  
{
  prev_receive_time_ = prev_receive_time;
}