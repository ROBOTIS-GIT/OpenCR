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

#include "../include/open_manipulator_libs/open_manipulator.h"

OpenManipulator::OpenManipulator()
{
  // Use Default OpenMANIPULATOR-X ID setting
  joint1_id = 11;
  joint2_id = 12;
  joint3_id = 13;
  joint4_id = 14;
  gripper_id = 15;
}
OpenManipulator::~OpenManipulator()
{
  delete kinematics_;
  delete joint_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void OpenManipulator::initOpenManipulator(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_loop_time)
{
  /*****************************************************************************
  ** Initialize Manipulator Parameters
  *****************************************************************************/
  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1",  // my name
           "world",   // parent name
           "joint2",  // child name
           math::vector3(0.012, 0.0, 0.017),                // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,    // axis of rotation
           joint1_id, // actuator id
           M_PI,      // max joint limit (3.14 rad)
           -M_PI);    // min joint limit (-3.14 rad)

  addJoint("joint2",  // my name
           "joint1",  // parent name
           "joint3",  // child name
           math::vector3(0.0, 0.0, 0.0595),                 // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,    // axis of rotation
           joint2_id, // actuator id
           M_PI_2,    // max joint limit (1.67 rad)
           -2.05);    // min joint limit (-2.05 rad)

  addJoint("joint3",  // my name
           "joint2",  // parent name
           "joint4",  // child name
           math::vector3(0.024, 0.0, 0.128),                // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,    // axis of rotation
           joint3_id, // actuator id
           1.53,      // max joint limit (1.53 rad)
           -M_PI_2);  // min joint limit (-1.67 rad)

  addJoint("joint4",  // my name
           "joint3",  // parent name
           "gripper", // child name
           math::vector3(0.124, 0.0, 0.0),                 // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,    // axis of rotation
           joint4_id, // actuator id
           2.0,       // max joint limit (2.0 rad)
           -1.8);     // min joint limit (-1.8 rad)

  addTool("gripper",  // my name
          "joint4",   // parent name
          math::vector3(0.126, 0.0, 0.0),                 // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          gripper_id, // actuator id
          0.010,      // max gripper limit (0.01 m)
          -0.010,     // min gripper limit (-0.01 m)
          -0.015);    // Convert unit from `meter` to `radian`

          
  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
  // kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);

  if (using_actual_robot_state)
  {
    /*****************************************************************************
    ** Initialize ㅓoint Actuator
    *****************************************************************************/
    // joint_ = new dynamixel::JointDynamixel();
    joint_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
    
    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(joint1_id);
    jointDxlId.push_back(joint2_id);
    jointDxlId.push_back(joint3_id);
    jointDxlId.push_back(joint4_id);
    addJointActuator(JOINT_DYNAMIXEL, joint_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);


    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new dynamixel::GripperDynamixel();

    // Set gripper actuator id 
    uint8_t gripperDxlId = gripper_id;
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);


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

void OpenManipulator::processOpenManipulator(double present_time)
{
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value  = getToolGoalValue();

  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);
  solveForwardKinematics();
}

// Use Custom ID settings
void OpenManipulator::setOpenManipulatorCustomJointId(uint8_t joint1, uint8_t joint2, uint8_t joint3, uint8_t joint4, uint8_t gripper)
{
  joint1_id = joint1;
  joint2_id = joint2;
  joint3_id = joint3;
  joint4_id = joint4;
  gripper_id = gripper;
}