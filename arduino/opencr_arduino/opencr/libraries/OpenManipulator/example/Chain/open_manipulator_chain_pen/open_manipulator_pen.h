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
#include "open_manipulator_pen_drawing.h"

#define CUSTOM_TRAJECTORY_SIZE 5
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"
#define DRAWING_ALPHABET          "drawing_alphabet"

#define JOINT_DYNAMIXEL "joint_dxl"

#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)

class OpenManipulatorPen : public robotis_manipulator::RobotisManipulator
{
private:
  robotis_manipulator::Kinematics *kinematics_;
  robotis_manipulator::JointActuator *actuator_;
  robotis_manipulator::ToolActuator *tool_;
  robotis_manipulator::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

 public:
  OpenManipulatorPen() {}
  virtual ~OpenManipulatorPen() {
    delete kinematics_;
    delete actuator_;
    delete tool_;
    for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
      delete custom_trajectory_[index];
  }

  void initOpenManipulator(bool using_actual_robot_state, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", float control_loop_time = 0.010)
  {
	/*****************************************************************************
	 ** Initialize Manipulator Parameter 
	*****************************************************************************/
	addWorld("world",   // world name
					"joint1"); // child name

	addJoint("joint1",  // my name
					"world",   // parent name
					"joint2",  // child name
					math::vector3(0.012, 0.0, 0.017),               // relative position
					math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
					Z_AXIS,    // axis of rotation
					11,        // actuator id
					M_PI,      // max joint limit (3.14 rad)
					-M_PI);    // min joint limit (-3.14 rad)

	addJoint("joint2",  // my name
					"joint1",  // parent name
					"joint3",  // child name
					math::vector3(0.0, 0.0, 0.0595),                // relative position
					math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
					Y_AXIS,    // axis of rotation
					12,        // actuator id
					M_PI_2,    // max joint limit (1.67 rad)
					-2.05);    // min joint limit (-2.05 rad)

	addJoint("joint3",  // my name
					"joint2",  // parent name
					"joint4",  // child name
					math::vector3(0.024, 0.0, 0.128),               // relative position
					math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
					Y_AXIS,    // axis of rotation
					13,        // actuator id
					1.53,      // max joint limit (1.53 rad)
					-M_PI_2);  // min joint limit (-1.67 rad)

	addJoint("joint4",  // my name
					"joint3",  // parent name
					"pen", // child name
					math::vector3(0.124, 0.0, 0.0),                 // relative position
					math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
					Y_AXIS,    // axis of rotation
					14,        // actuator id
					2.0,       // max joint limit (2.0 rad)
					-1.8);     // min joint limit (-1.8 rad)
            
	addTool("pen",   // my name
					"joint4", // parent name
					math::vector3(0.043, 0.0, 0.0), // relative position
					math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
					-1); // actuator id

  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

    if(using_actual_robot_state)
    {
			/*****************************************************************************
			** Initialize joint Actuator
			*****************************************************************************/
			// actuator_ = new dynamixel::JointDynamixel();
			actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);

      // communication setting argument
      STRING dxl_comm_arg[2] = {usb_port, baud_rate};
      void *p_dxl_comm_arg = &dxl_comm_arg;

      // set joint actuator id
      std::vector<uint8_t> jointDxlId;
      jointDxlId.push_back(11);
      jointDxlId.push_back(12);
      jointDxlId.push_back(13);
      jointDxlId.push_back(14);
      addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

      // set joint actuator control mode
      STRING joint_dxl_mode_arg = "position_mode";
      void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
      setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

      // set joint actuator parameter
      STRING joint_dxl_opt_arg[2] = {"Position_P_Gain", "1200"};
      void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
      setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

      // all actuator enable
      enableAllActuator();
      receiveAllJointActuatorValue();
    }

		/*****************************************************************************
		** Initialize Custom Trajectory
		*****************************************************************************/
		custom_trajectory_[0] = new custom_trajectory::Line();
		custom_trajectory_[1] = new custom_trajectory::Circle();
		custom_trajectory_[2] = new custom_trajectory::Rhombus();
		custom_trajectory_[3] = new custom_trajectory::Heart();
		custom_trajectory_[4] = new open_manipulator_pen_drawing::Alphabet();

		addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
		addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
		addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
		addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
		addCustomTrajectory(DRAWING_ALPHABET, custom_trajectory_[4]);
		
  }

  void processOpenManipulator(double present_time)
  {
    JointWaypoint goal_joint_value  = getJointGoalValueFromTrajectory(present_time);

    receiveAllJointActuatorValue();
    if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
    solveForwardKinematics();
  }
};

#endif // OPEN_MANIPULATOR_PEN_H_




