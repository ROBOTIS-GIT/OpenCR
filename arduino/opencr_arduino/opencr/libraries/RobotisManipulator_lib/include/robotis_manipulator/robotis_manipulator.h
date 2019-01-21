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
#include "robotis_manipulator_debug.h"

#include <algorithm>

namespace ROBOTIS_MANIPULATOR
{

class RobotisManipulator
{
private:
  Manipulator manipulator_;
  Trajectory trajectory_;
  bool trajectory_initialization;

  std::map<Name, JointActuator *> joint_actuator_;
  std::map<Name, ToolActuator *> tool_actuator_;

  Kinematics *kinematics_;

  bool using_platform_;
  bool moving_;
  bool step_moving_;

private:
  void startMoving();

  JointWayPoint getTrajectoryJointValue(double tick_time);

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();

  /////////////////////////// initialize function /////////////////////////////
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(3),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

  void addJoint(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Eigen::Vector3d relative_position,
                    Eigen::Matrix3d relative_orientation,
                    Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                    int8_t joint_actuator_id = -1, double max_limit = M_PI, double min_limit = -M_PI,
                    double coefficient = 1.0,
                    double mass = 0.0,
                    Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
                    Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, double max_limit =M_PI, double min_limit = -M_PI,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  void addKinematics(Kinematics *kinematics);
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg);
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator, uint8_t id, const void *arg);
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);

  // Manipulator
  Manipulator *getManipulator();

  JointValue getJointValue(Name joint_name);
  JointValue getToolValue(Name tool_name);
  std::vector<JointValue> getAllActiveJointValue();
  std::vector<JointValue> getAllJointValue();
  std::vector<double> getAllToolPosition();
  std::vector<JointValue> getAllToolValue();
  KinematicPose getKinematicPose(Name component_name);
  DynamicPose getDynamicPose(Name component_name);
  PoseValue getPoseValue(Name component_name);

  // Kinematics (include virtual function)
  Eigen::MatrixXd jacobian(Name tool_name);
  void forwardKinematics();
  bool inverseKinematics(Name tool_name, PoseValue goal_pose, std::vector<JointValue> *goal_joint_value);
  void kinematicsSetOption(const void* arg);

  // Actuator (include virtual function)
  void jointActuatorSetMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void toolActuatorSetMode(Name actuator_name, const void *arg);

  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  uint8_t getToolActuatorId(Name actuator_name);

  void actuatorEnable(Name actuator_name);
  void actuatorDisable(Name actuator_name);
  void allJointActuatorEnable();
  void allJointActuatorDisable();
  void allToolActuatorEnable();
  void allToolActuatorDisable();
  void allActuatorEnable();
  void allActuatorDisable();

  bool isEnabled(Name actuator_name);

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

  ////////////////////////////////////////trajectory

  // Time
  double getTrajectoryMoveTime();
  bool isMoving();

  //Joint limit (Check as Trajectory Manipulator)
  bool checkLimit(Name component_name, double position);
  bool checkLimit(Name component_name, JointValue value);
  bool checkLimit(std::vector<Name> component_name, std::vector<double> position_vector);
  bool checkLimit(std::vector<Name> component_name, std::vector<JointValue> value_vector);

  //Trajectory control move fuction
  void jointTrajectoryMoveFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void jointTrajectoryMove(std::vector<double> goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void jointTrajectoryMove(std::vector<JointValue> goal_joint_value, double move_time, std::vector<JointValue> present_joint_value = {});
  void jointTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void jointTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  void jointTrajectoryMove(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  void taskTrajectoryMoveFromPresentPose(Name tool_name, Eigen::Vector3d position_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  void taskTrajectoryMoveFromPresentPose(Name tool_name, Eigen::Matrix3d orientation_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  void taskTrajectoryMoveFromPresentPose(Name tool_name, KinematicPose goal_pose_delta, double move_time, std::vector<JointValue> present_joint_value = {});
  void taskTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  void taskTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  void taskTrajectoryMove(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  void customTrajectorysetOption(Name trajectory_name, const void* arg);
  void customTrajectoryMove(Name trajectory_name, Name tool_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  void customTrajectoryMove(Name trajectory_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});

  void toolMove(Name tool_name, double tool_goal_position);

  void TrajectoryWait(double wait_time, std::vector<JointValue> present_joint_value = {});

  // Additional functions
  std::vector<JointValue> getJointGoalValueFromTrajectory(double present_time);
  std::vector<JointValue> getToolGoalValue();
  std::vector<JointValue> getJointGoalValueFromTrajectoryTickTime(double tick_time);
};
} // namespace ROBOTIS_MANIPULATOR

#endif
