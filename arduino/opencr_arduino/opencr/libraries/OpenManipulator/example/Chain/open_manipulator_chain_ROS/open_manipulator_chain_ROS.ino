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

#include "open_manipulator_chain_ROS.h"

double control_time = 0.010f;

void setup()
{
  DEBUG.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.advertise(joint_states_pub);  
  nh.advertise(kinematic_pose_pub);  
  nh.advertise(open_manipulator_states_pub);  

  nh.advertiseService(goal_joint_space_path_server);
  nh.advertiseService(goal_joint_space_path_to_kinematics_pose_server);
  nh.advertiseService(goal_joint_space_path_to_kinematics_position_server);
  nh.advertiseService(goal_joint_space_path_to_kinematics_orientation_server);
  nh.advertiseService(goal_task_space_path_server);
  nh.advertiseService(goal_task_space_path_position_only_server);
  nh.advertiseService(goal_task_space_path_orientation_only_server);
  nh.advertiseService(goal_joint_space_path_from_present_server);
  nh.advertiseService(goal_task_space_path_from_present_server);
  nh.advertiseService(goal_task_space_path_from_present_position_only_server);
  nh.advertiseService(goal_task_space_path_from_present_orientation_only_server);
  nh.advertiseService(goal_tool_control_server);
  nh.advertiseService(set_actuator_state_server);
  nh.advertiseService(goal_drawing_trajectory_server);

  // Initialize Open Manipulator.  
  open_manipulator.initOpenManipulator(true);

  log::println("OpenManipulator Debugging Port");
}

void loop()
{
  present_time = (float)(millis()/1000.0f);
  nh.spinOnce();

  if(present_time-previous_time >= control_time)
  {
    open_manipulator.processOpenManipulator(millis()/1000.0);
    previous_time = (float)(millis()/1000.0f);
  }
  if(present_time - previous_time_pub >= control_time*10)
  {
    publishJointStates();
    publishKinematicPose();
    publishOpenManipulatorStates();
    previous_time_pub = (float)(millis()/1000.0f);
  }
}

/*******************************************************************************
* Service server (set trajectory using joint angle values)
*******************************************************************************/
void goalJointSpacePathCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res)
{
  std::vector <double> target_angle;
  for(int i = 0; i < req.joint_position.joint_name_length; i ++)
    target_angle.push_back(req.joint_position.position[i]);

  open_manipulator.makeJointTrajectory(target_angle, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic pose value)
*******************************************************************************/
void goalJointSpacePathToKinematicsPoseCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic position value)
*******************************************************************************/
void goalJointSpacePathToKinematicsPositionCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic orientation value)
*******************************************************************************/
void goalJointSpacePathToKinematicsOrientationCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic pose value)
*******************************************************************************/
void goalTaskSpacePathCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeTaskTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic position value)
*******************************************************************************/
void goalTaskSpacePathPositionOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator.makeTaskTrajectory(req.end_effector_name, position, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic orientation value)
*******************************************************************************/
void goalTaskSpacePathOrientationOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                        req.kinematics_pose.pose.orientation.x,
                        req.kinematics_pose.pose.orientation.y,
                        req.kinematics_pose.pose.orientation.z);
  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeTaskTrajectory(req.end_effector_name, orientation, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using joint angle values form present values)
*******************************************************************************/
void goalJointSpacePathFromPresentCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res)
{
  std::vector <double> target_angle;
  for(int i = 0; i < req.joint_position.joint_name_length; i ++)
    target_angle.push_back(req.joint_position.position[i]);

  open_manipulator.makeJointTrajectoryFromPresentPosition(target_angle, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic pose value from present value)
*******************************************************************************/
void goalTaskSpacePathFromPresentCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeTaskTrajectoryFromPresentPose(req.planning_group, target_pose, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic position value from present value)
*******************************************************************************/
void goalTaskSpacePathFromPresentPositionOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator.makeTaskTrajectoryFromPresentPose(req.planning_group, position, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set trajectory using kinematic oreintation value from present value)
*******************************************************************************/
void goalTaskSpacePathFromPresentOrientationOnlyCallback(const SetKinematicsPose::Request & req, SetKinematicsPose::Response & res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                        req.kinematics_pose.pose.orientation.x,
                        req.kinematics_pose.pose.orientation.y,
                        req.kinematics_pose.pose.orientation.z);
  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator.makeTaskTrajectoryFromPresentPose(req.planning_group, orientation, req.path_time);

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set tool control)
*******************************************************************************/
void goalToolControlCallback(const SetJointPosition::Request & req, SetJointPosition::Response & res)
{
  for(int i = 0; i < req.joint_position.joint_name_length; i ++)
  {
    open_manipulator.makeToolTrajectory(req.joint_position.joint_name[i], req.joint_position.position[i]);
  }

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set actuator state)
*******************************************************************************/
void setActuatorStateCallback(const SetActuatorState::Request & req, SetActuatorState::Response & res)
{
  if(req.set_actuator_state == true) // torque on
    open_manipulator.enableAllActuator();
  else // torque off
    open_manipulator.disableAllActuator();

  res.is_planned = true;
}

/*******************************************************************************
* Service server (set drawing trajectory)
*******************************************************************************/
void goalDrawingTrajectoryCallBack(const SetDrawingTrajectory::Request & req, SetDrawingTrajectory::Response & res)
{
  STRING trajectory_name (req.drawing_trajectory_name);

  if(trajectory_name == "circle")
  {
    double draw_circle_arg[3];
    draw_circle_arg[0] = req.param[0];  // radius (m)
    draw_circle_arg[1] = req.param[1];  // revolution (rev)
    draw_circle_arg[2] = req.param[2];  // start angle position (rad)
    void* p_draw_circle_arg = &draw_circle_arg;

    open_manipulator.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time);
  }
  else if(trajectory_name == "line")
  {
    TaskWaypoint draw_line_arg;
    draw_line_arg.kinematic.position(0) = req.param[0]; // x axis (m)
    draw_line_arg.kinematic.position(1) = req.param[1]; // y axis (m)
    draw_line_arg.kinematic.position(2) = req.param[2]; // z axis (m)
    void *p_draw_line_arg = &draw_line_arg;

    open_manipulator.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req.end_effector_name, p_draw_line_arg, req.path_time);
  }
  else if(trajectory_name == "rhombus")
  {
    double draw_rhombus_arg[3];
    draw_rhombus_arg[0] = req.param[0];  // radius (m)
    draw_rhombus_arg[1] = req.param[1];  // revolution (rev)
    draw_rhombus_arg[2] = req.param[2];  // start angle position (rad)
    void* p_draw_rhombus_arg = &draw_rhombus_arg;

    open_manipulator.makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req.end_effector_name, p_draw_rhombus_arg, req.path_time);
  }
  else if(trajectory_name == "heart")
  {
    double draw_heart_arg[3];
    draw_heart_arg[0] = req.param[0];  // radius (m)
    draw_heart_arg[1] = req.param[1];  // revolution (rev)
    draw_heart_arg[2] = req.param[2];  // start angle position (rad)
    void* p_draw_heart_arg = &draw_heart_arg;

    open_manipulator.makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req.end_effector_name, p_draw_heart_arg, req.path_time);
  }
  res.is_planned = true;
}

/*******************************************************************************
* Publish msgs (joint states)
*******************************************************************************/
void publishJointStates(void)
{
  auto joints_name = open_manipulator.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator.getAllActiveJointValue();
  auto tool_value = open_manipulator.getAllToolValue();

  joint_states_msg.name_length = joints_name.size()+tool_name.size();
  joint_states_msg.position_length = joints_name.size()+tool_name.size();
  joint_states_msg.velocity_length = joints_name.size()+tool_name.size();
  joint_states_msg.effort_length = joints_name.size()+tool_name.size();

  char* pub_joint_name[joints_name.size()+tool_name.size()];
  float pub_joint_position[joints_name.size()+tool_name.size()] = {0.0, };
  float pub_joint_velocity[joints_name.size()+tool_name.size()] = {0.0, };
  float pub_joint_effort[joints_name.size()+tool_name.size()] = {0.0, };
  
  for(uint8_t i = 0; i < joints_name.size(); i ++)
  {
    pub_joint_name[i] = const_cast<char*>(joints_name[i].c_str());
    pub_joint_position[i] = (float)joint_value[i].position;
    pub_joint_velocity[i] = (float)joint_value[i].velocity;
    pub_joint_effort[i] = (float)joint_value[i].effort;
  }

  for(uint8_t i = joints_name.size(); i < joints_name.size()+tool_name.size(); i ++)
  {
    pub_joint_name[i] = const_cast<char*>(tool_name[i-joints_name.size()].c_str());
    pub_joint_position[i] = tool_value[i-joints_name.size()].position;
    pub_joint_velocity[i] = 0.0f;
    pub_joint_effort[i] = 0.0f;
  }

  joint_states_msg.header.stamp = nh.now();
  joint_states_msg.name = pub_joint_name;
  joint_states_msg.position = pub_joint_position;
  joint_states_msg.velocity = pub_joint_velocity;
  joint_states_msg.effort = pub_joint_effort;

  joint_states_pub.publish(&joint_states_msg);
}

/*******************************************************************************
* Publish msgs (kinematic pose)
*******************************************************************************/
void publishKinematicPose(void)
{
  auto opm_tools_name = open_manipulator.getManipulator()->getAllToolComponentName();

  KinematicPose pose = open_manipulator.getKinematicPose("gripper");
  kinematic_pose_msg.pose.position.x = pose.position[0];
  kinematic_pose_msg.pose.position.y = pose.position[1];
  kinematic_pose_msg.pose.position.z = pose.position[2];
  Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
  kinematic_pose_msg.pose.orientation.w = orientation.w();
  kinematic_pose_msg.pose.orientation.x = orientation.x();
  kinematic_pose_msg.pose.orientation.y = orientation.y();
  kinematic_pose_msg.pose.orientation.z = orientation.z();

  kinematic_pose_pub.publish(&kinematic_pose_msg);
}
/*******************************************************************************
* Publish msgs (openmanipulator states)
*******************************************************************************/
void publishOpenManipulatorStates(void)
{
  if(open_manipulator.getMovingState())
    open_manipulator_states_msg.open_manipulator_moving_state = open_manipulator_states_msg.IS_MOVING;
  else
    open_manipulator_states_msg.open_manipulator_moving_state = open_manipulator_states_msg.STOPPED;

  if(open_manipulator.getActuatorEnabledState(JOINT_DYNAMIXEL))
    open_manipulator_states_msg.open_manipulator_actuator_state = open_manipulator_states_msg.ACTUATOR_ENABLED;
  else
    open_manipulator_states_msg.open_manipulator_actuator_state = open_manipulator_states_msg.ACTUATOR_DISABLED;

  open_manipulator_states_pub.publish(&open_manipulator_states_msg);
}