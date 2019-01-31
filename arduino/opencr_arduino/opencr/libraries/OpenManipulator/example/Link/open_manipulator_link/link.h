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

#ifndef LINK_H_
#define LINK_H_

// Necessary library
#include <open_manipulator_libs.h>

// User-defined library
#include "link_kinematics.h"
#include "vacuum_actuator.h"

/////////////custom trajectory///////////////
#define CUSTOM_TRAJECTORY_SIZE 4
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"
////////////////////////////////////////////

/////////////control time set////////////////
#define ACTUATOR_CONTROL_TIME 0.010
////////////////////////////////////////////

//////////////Vacuum Pin Num////////////////
#define RELAY_PIN 8
////////////////////////////////////////////

////////////////////////////////////////////
#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)
////////////////////////////////////////////

class OpenManipulatorLink : public robotis_manipulator::RobotisManipulator
{
 private:
  robotis_manipulator::Kinematics *kinematics_;
  robotis_manipulator::JointActuator *joint_actuator_;
  robotis_manipulator::ToolActuator *tool_actuator_;
  robotis_manipulator::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

  // bool processing_;
  std::vector<uint8_t> jointDxlId;

 public:
  OpenManipulatorLink(){}
  virtual ~OpenManipulatorLink(){}

  void initManipulator(bool using_platform, STRING usb_port = "/dev/ttyACM0", STRING baud_rate = "1000000", float control_loop_time = 0.010)
  {
    /*****************************************************************************
    ** Initialize Manipulator Parameter 
    *****************************************************************************/
    addWorld("world", "joint01");
    addJoint("joint01", "world", "joint02",
                            math::vector3(-0.23867882, 0, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Z_AXIS,
                            1,
                            M_PI,
                            -M_PI,
                            1.0);
    addComponentChild("joint01", "joint03");
    addComponentChild("joint01", "joint08");
    addJoint("joint02", "joint01", "joint06", 
                            math::vector3(0, 0.022, 0.052),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS,
                            2,
                            0.0,
                            -M_PI,
                            -1.0
                            );
    addJoint("joint03", "joint01", "joint04",
                            math::vector3(0, -0.022, 0.052),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS,
                            3,
                            -M_PI/4,
                            -M_PI,
                            1.0);
    addJoint("joint04", "joint03", "joint05",
                            math::vector3(0.050, 0.007, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint05", "joint04", "joint06",
                            math::vector3(0.200, 0.006, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint06", "joint02", "joint07",
                            math::vector3(0.200, -0.016, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint07", "joint06", "vacuum",
                            math::vector3(0.200, -0.009, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint08", "joint01", "joint09",
                            math::vector3(-0.04531539, 0.006, 0.07313091),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint09", "joint08", "joint10",
                            math::vector3(0.200, 0.009, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint10", "joint09", "joint11",
                            math::vector3(0.07660444, -0.006, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addJoint("joint11", "joint10", "vacuum",
                            math::vector3(0.200, -0.006, 0),
                            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                            Y_AXIS);
    addTool("vacuum", "joint07",
                        math::vector3(0.03867882, 0.003, -0.01337315-0.01),
                        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0),
                        4,
                        1.0);

    /*****************************************************************************
    ** Initialize Kinematics 
    *****************************************************************************/
    kinematics_ = new kinematics::Link();
    addKinematics(kinematics_);

    if(using_platform)
    {
      /*****************************************************************************
      ** Initialize ㅓoint Actuator
      *****************************************************************************/
      // actuator_ = new dynamixel::JointDynamixel();
      joint_actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
      
      // Set communication arguments
      STRING dxl_comm_arg[2] = {usb_port, baud_rate};
      void *p_dxl_comm_arg = &dxl_comm_arg;

      // Set joint actuator id
      std::vector<uint8_t> jointDxlId;
      jointDxlId.push_back(1);
      jointDxlId.push_back(2);
      jointDxlId.push_back(3);
      addJointActuator("joint_dxl", joint_actuator_, jointDxlId, p_dxl_comm_arg);

      // Set joint actuator control mode
      STRING joint_dxl_mode_arg = "position_mode";
      void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
      setJointActuatorMode("joint_dxl", jointDxlId, p_joint_dxl_mode_arg);


    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
      tool_actuator_ = new VacuumModule();
      uint8_t suc_pin_arg = RELAY_PIN;
      void *p_suc_pin_arg = &suc_pin_arg;
      addToolActuator("vacuum_module", tool_actuator_, getManipulator()->getId("vacuum"), p_suc_pin_arg);
    
      // Enable All Actuators 
      enableAllActuator();

      // Receive current angles from all actuators 
      receiveAllJointActuatorValue();
      receiveAllToolActuatorValue();
    }

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
  }

  void Process(double present_time)
  {
    JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
    JointWaypoint goal_tool_value  = getToolGoalValue();

    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
    if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
    if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);
    solveForwardKinematics();
  } 
};

#endif //LINK_H_