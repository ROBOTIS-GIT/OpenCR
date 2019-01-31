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

#ifndef OPEN_MANIPULATOR_CHAIN_ROS_H_
#define OPEN_MANIPULATOR_CHAIN_ROS_H_

#include <ros.h>
#include <ros/time.h>

#include <open_manipulator_libs.h>

#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/KinematicsPose.h>
#include <open_manipulator_msgs/OpenManipulatorState.h>

#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/SetActuatorState.h>
#include <open_manipulator_msgs/SetDrawingTrajectory.h>

using open_manipulator_msgs::SetJointPosition;
using open_manipulator_msgs::SetKinematicsPose;
using open_manipulator_msgs::SetActuatorState;
using open_manipulator_msgs::SetDrawingTrajectory;

OpenManipulator open_manipulator;
double present_time = 0.0;
double previous_time = 0.0;
double previous_time_pub = 0.0;

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Service Server
*******************************************************************************/
void goalJointSpacePathCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res);
ros::ServiceServer<SetJointPosition::Request, SetJointPosition::Response> goal_joint_space_path_server(
  "open_manipulator/goal_joint_space_path", &goalJointSpacePathCallback);

void goalJointSpacePathToKinematicsPoseCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_joint_space_path_to_kinematics_pose_server(
  "open_manipulator/goal_joint_space_path_to_kinematics_pose", &goalJointSpacePathToKinematicsPoseCallback);

void goalJointSpacePathToKinematicsPositionCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_joint_space_path_to_kinematics_position_server(
  "open_manipulator/goal_joint_space_path_to_kinematics_position", &goalJointSpacePathToKinematicsPositionCallback);

void goalJointSpacePathToKinematicsOrientationCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_joint_space_path_to_kinematics_orientation_server(
  "open_manipulator/goal_joint_space_path_to_kinematics_orientation", &goalJointSpacePathToKinematicsOrientationCallback);

void goalTaskSpacePathCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_server(
  "open_manipulator/goal_task_space_path", &goalTaskSpacePathCallback);

void goalTaskSpacePathPositionOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_position_only_server(
  "open_manipulator/goal_task_space_path_position_only", &goalTaskSpacePathPositionOnlyCallback);

void goalTaskSpacePathOrientationOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_orientation_only_server(
  "open_manipulator/goal_task_space_path_orientation_only", &goalTaskSpacePathOrientationOnlyCallback);

void goalJointSpacePathFromPresentCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res);
ros::ServiceServer<SetJointPosition::Request, SetJointPosition::Response> goal_joint_space_path_from_present_server(
  "open_manipulator/goal_joint_space_path_from_present", &goalJointSpacePathFromPresentCallback);

void goalTaskSpacePathFromPresentCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_from_present_server(
  "open_manipulator/goal_task_space_path_from_present", &goalTaskSpacePathFromPresentCallback);

void goalTaskSpacePathFromPresentPositionOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_from_present_position_only_server(
  "open_manipulator/goal_task_space_path_from_present_position_only", &goalTaskSpacePathFromPresentPositionOnlyCallback);

void goalTaskSpacePathFromPresentOrientationOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
ros::ServiceServer<SetKinematicsPose::Request, SetKinematicsPose::Response> goal_task_space_path_from_present_orientation_only_server(
  "open_manipulator/goal_task_space_path_from_present_orientation_only", &goalTaskSpacePathFromPresentOrientationOnlyCallback);

void goalToolControlCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res);
ros::ServiceServer<SetJointPosition::Request, SetJointPosition::Response> goal_tool_control_server(
  "open_manipulator/goal_tool_control", &goalToolControlCallback);

void setActuatorStateCallback(const SetActuatorState::Request & req, SetActuatorState::Response & res);
ros::ServiceServer<SetActuatorState::Request, SetActuatorState::Response> set_actuator_state_server(
  "open_manipulator/set_actuator_state", &setActuatorStateCallback);

void goalDrawingTrajectoryCallBack(const SetDrawingTrajectory::Request & req, SetDrawingTrajectory::Response & res);
ros::ServiceServer<SetDrawingTrajectory::Request, SetDrawingTrajectory::Response> goal_drawing_trajectory_server(
  "open_manipulator/goal_drawing_trajectory", &goalDrawingTrajectoryCallBack);

/*******************************************************************************
* Publisher
*******************************************************************************/
void publishJointStates(void);
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("open_manipulator/joint_states", &joint_states_msg);

void publishKinematicPose(void);
open_manipulator_msgs::KinematicsPose kinematic_pose_msg;
ros::Publisher kinematic_pose_pub("open_manipulator/gripper/kinematics_pose", &kinematic_pose_msg);

void publishOpenManipulatorStates(void);
open_manipulator_msgs::OpenManipulatorState open_manipulator_states_msg; 
ros::Publisher open_manipulator_states_pub("open_manipulator/states", &open_manipulator_states_msg);

#endif
