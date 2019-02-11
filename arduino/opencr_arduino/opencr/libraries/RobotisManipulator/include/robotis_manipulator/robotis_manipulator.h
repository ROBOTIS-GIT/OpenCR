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

#ifndef ROBOTIS_MANIPULATOR_H_
#define ROBOTIS_MANIPULATOR_H_

#include "robotis_manipulator_common.h"
#include "robotis_manipulator_manager.h"
#include "robotis_manipulator_trajectory_generator.h"
#include "robotis_manipulator_math.h"
#include "robotis_manipulator_log.h"

#include <algorithm>

namespace robotis_manipulator
{

class RobotisManipulator
{
private:
  Manipulator manipulator_;
  Trajectory trajectory_;
  Kinematics *kinematics_;
  std::map<Name, JointActuator *> joint_actuator_;
  std::map<Name, ToolActuator *> tool_actuator_;

  bool trajectory_initialized_state_;
  bool actuator_added_stete_;
  bool moving_state_;
  bool step_moving_state_;

private:
  void startMoving();

  JointWaypoint getTrajectoryJointValue(double tick_time);

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();


  /*****************************************************************************
  ** Initialize Function
  *****************************************************************************/
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

  void addJoint(Name my_name,
                Name parent_name,
                Name child_name,
                Eigen::Vector3d relative_position,
                Eigen::Matrix3d relative_orientation,
                Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                int8_t joint_actuator_id = -1, 
                double max_position_limit = M_PI, 
                double min_position_limit = -M_PI,
                double coefficient = 1.0,
                double mass = 0.0,
                Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, 
               double max_position_limit =M_PI, 
               double min_position_limit = -M_PI,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void printManipulatorSetting();

  void addKinematics(Kinematics *kinematics);
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg);
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator, uint8_t id, const void *arg);
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);


  /*****************************************************************************
  ** Manipulator Function
  *****************************************************************************/
  Manipulator *getManipulator();

  JointValue getJointValue(Name joint_name);
  JointValue getToolValue(Name tool_name);
  std::vector<JointValue> getAllActiveJointValue();
  std::vector<JointValue> getAllJointValue();
  std::vector<double> getAllToolPosition();
  std::vector<JointValue> getAllToolValue();
  KinematicPose getKinematicPose(Name component_name);
  DynamicPose getDynamicPose(Name component_name);
  Pose getPose(Name component_name);


  /*****************************************************************************
  ** Kinematics Function (Including Virtual Function)
  *****************************************************************************/
  Eigen::MatrixXd jacobian(Name tool_name);
  void solveForwardKinematics();
  bool solveInverseKinematics(Name tool_name, Pose goal_pose, std::vector<JointValue> *goal_joint_value);
  void setKinematicsOption(const void* arg);


  /*****************************************************************************
  ** Actuator Function (Including Virtual Function)
  *****************************************************************************/
  void setJointActuatorMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void setToolActuatorMode(Name actuator_name, const void *arg);
  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  uint8_t getToolActuatorId(Name actuator_name);
  void enableActuator(Name actuator_name);
  void disableActuator(Name actuator_name);
  void enableAllJointActuator();
  void disableAllJointActuator();
  void enableAllToolActuator();
  void disableAllToolActuator();
  void enableAllActuator();
  void disableAllActuator();
  bool getActuatorEnabledState(Name actuator_name);

  bool sendJointActuatorValue(Name joint_component_name, JointValue value);
  bool sendMultipleJointActuatorValue(std::vector<Name> joint_component_name, std::vector<JointValue> value_vector);
  bool sendAllJointActuatorValue(std::vector<JointValue> value_vector);
  JointValue receiveJointActuatorValue(Name joint_component_name);
  std::vector<JointValue> receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name);
  std::vector<JointValue> receiveAllJointActuatorValue();

  bool sendToolActuatorValue(Name tool_component_name, JointValue value);
  bool sendMultipleToolActuatorValue(std::vector<Name> tool_component_name, std::vector<JointValue> value_vector);
  bool sendAllToolActuatorValue(std::vector<JointValue> value_vector);
  JointValue receiveToolActuatorValue(Name tool_component_name);
  std::vector<JointValue> receiveMultipleToolActuatorValue(std::vector<Name> tool_component_name);
  std::vector<JointValue> receiveAllToolActuatorValue();


  /*****************************************************************************
  ** Time Function
  *****************************************************************************/
  double getTrajectoryMoveTime();
  bool getMovingState();


  /*****************************************************************************
  ** Check Joint Limit Function
  *****************************************************************************/
  bool checkJointLimit(Name component_name, double position);
  bool checkJointLimit(Name component_name, JointValue value);
  bool checkJointLimit(std::vector<Name> component_name, std::vector<double> position_vector);
  bool checkJointLimit(std::vector<Name> component_name, std::vector<JointValue> value_vector);


  /*****************************************************************************
  ** Trajectory Control Fuction
  *****************************************************************************/
  Trajectory *getTrajectory();

  void makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeJointTrajectory(std::vector<double> goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeJointTrajectory(std::vector<JointValue> goal_joint_value, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeJointTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeJointTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeJointTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  void makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Vector3d position_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Matrix3d orientation_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeTaskTrajectoryFromPresentPose(Name tool_name, KinematicPose goal_pose_delta, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeTaskTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeTaskTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeTaskTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  void makeCustomTrajectory(Name trajectory_name, Name tool_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  void makeCustomTrajectory(Name trajectory_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});

  void sleepTrajectory(double wait_time, std::vector<JointValue> present_joint_value = {});

  void makeToolTrajectory(Name tool_name, double tool_goal_position);

  std::vector<JointValue> getJointGoalValueFromTrajectory(double present_time);
  std::vector<JointValue> getToolGoalValue();
  std::vector<JointValue> getJointGoalValueFromTrajectoryTickTime(double tick_time);
};
} // namespace ROBOTIS_MANIPULATOR

#endif
