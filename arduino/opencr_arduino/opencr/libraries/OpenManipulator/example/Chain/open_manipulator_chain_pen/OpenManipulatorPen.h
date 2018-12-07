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

#ifndef OPEN_MANIPULATOR_PEN_H_
#define OPEN_MANIPULATOR_PEN_H_

#include <open_manipulator_libs.h>

#define NUM_OF_JOINT 4
#define DXL_SIZE 4

#define DRAWING_LINE "drawing_line"
#define DRAWING_CIRCLE "drawing_circle"
#define DRAWING_RHOMBUS "drawing_rhombus"
#define DRAWING_HEART "drawing_heart"

#define JOINT_DYNAMIXEL "joint_dxl"

#define CONTROL_TIME 0.010 //s

#define X_AXIS RM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS RM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS RM_MATH::makeVector3(0.0, 0.0, 1.0)

class OPEN_MANIPULATOR_PEN : public ROBOTIS_MANIPULATOR::RobotisManipulator
{
private:
  ROBOTIS_MANIPULATOR::Kinematics *kinematics_;
  ROBOTIS_MANIPULATOR::JointActuator *actuator_;
  ROBOTIS_MANIPULATOR::ToolActuator *tool_;

  DRAWING::Line line_;
  DRAWING::Circle circle_;
  DRAWING::Rhombus rhombus_;
  DRAWING::Heart heart_;

  bool platform_;
  std::vector<uint8_t> jointDxlId;
 public:
  OPEN_MANIPULATOR_PEN() {}
  virtual ~OPEN_MANIPULATOR_PEN() {}

  void initManipulator(bool using_platform, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000")
  {
    platform_ = using_platform;
    ////////// manipulator parameter initialization

    addWorld("world",   // world name
            "joint1"); // child name

    addJoint("joint1", // my name
            "world",  // parent name
            "joint2", // child name
            RM_MATH::makeVector3(0.012, 0.0, 0.017), // relative position
            RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
            Z_AXIS, // axis of rotation
            11,     // actuator id
            M_PI,   // max joint limit (3.14 rad)
            -M_PI); // min joint limit (-3.14 rad)


    addJoint("joint2", // my name
            "joint1", // parent name
            "joint3", // child name
            RM_MATH::makeVector3(0.0, 0.0, 0.058), // relative position
            RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS, // axis of rotation
            12,     // actuator id
            M_PI_2,   // max joint limit (1.67 rad)
            -2.05); // min joint limit (-2.05 rad)

    addJoint("joint3", // my name
            "joint2", // parent name
            "joint4", // child name
            RM_MATH::makeVector3(0.024, 0.0, 0.128), // relative position
            RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS, // axis of rotation
            13,     // actuator id
            1.53,      // max joint limit (1.53 rad)
            -M_PI_2); // min joint limit (-1.67 rad)

    addJoint("joint4", // my name
            "joint3", // parent name
            "pen",   // child name
            RM_MATH::makeVector3(0.124, 0.0, 0.0), // relative position
            RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS, // axis of rotation
            14,     // actuator id
            2.0,    // max joint limit (2.0 rad)
            -1.8);  // min joint limit (-1.8 rad)
            
    addTool("pen",   // my name
            "joint4", // parent name
            RM_MATH::makeVector3(0.043, 0.0, 0.0), // relative position
            RM_MATH::convertRPYToRotation(0.0, 0.0, 0.0), // relative orientation
            15); // actuator id

    ////////// kinematics init.
    kinematics_ = new KINEMATICS::Chain();
    addKinematics(kinematics_);
    STRING inverse_option[2] = {"inverse_solver", "chain_custum_inverse_kinematics"};
  //  STRING inverse_option[2] = {"inverse_solver", "sr_inverse"};
  //  STRING inverse_option[2] = {"inverse_solver", "position_only_inverse"};
  //  STRING inverse_option[2] = {"inverse_solver", "normal_inverse"};
    void *inverse_option_arg = &inverse_option;
    kinematicsSetOption(inverse_option_arg);

    if(platform_)
    {
      ////////// joint actuator init.
      actuator_ = new DYNAMIXEL::JointDynamixel();
      // communication setting argument
      STRING dxl_comm_arg[2] = {usb_port, baud_rate};
      void *p_dxl_comm_arg = &dxl_comm_arg;

      // set joint actuator id
      jointDxlId.push_back(11);
      jointDxlId.push_back(12);
      jointDxlId.push_back(13);
      jointDxlId.push_back(14);

      addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

      // set joint actuator parameter
      STRING joint_dxl_opt_arg[2] = {"Return_Delay_Time", "0"};
      void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
      jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

      // set joint actuator control mode
      STRING joint_dxl_mode_arg = "position_mode";
      void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
      jointActuatorSetMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

      // all actuator enable
      allActuatorEnable();
      receiveAllJointActuatorValue();
    }
    ////////// drawing path
    addDrawingTrajectory(DRAWING_LINE, &line_);
    addDrawingTrajectory(DRAWING_CIRCLE, &circle_);
    addDrawingTrajectory(DRAWING_RHOMBUS, &rhombus_);
    addDrawingTrajectory(DRAWING_HEART, &heart_);

    ////////// manipulator trajectory & control time initialization
    setTrajectoryControlTime(CONTROL_TIME);
  }

  void openManipulatorProcess(double present_time)
  {
    std::vector<WayPoint> goal_value  = getJointGoalValueFromTrajectory(present_time);

    if(platform_)
    {
      receiveAllJointActuatorValue();
      if(goal_value.size() != 0) sendAllJointActuatorValue(goal_value);
    }
    else // visualization
    {
      if(goal_value.size() != 0) setAllActiveJointWayPoint(goal_value);
    }
    forwardKinematics();
  }

  bool getPlatformFlag()
  {
    return platform_;
  }

};

#endif // OPEN_MANIPULATOR_PEN_H_




