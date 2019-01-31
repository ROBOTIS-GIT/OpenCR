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

#ifndef ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_
#define ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_

#include <math.h>
#include <vector>

#include "robotis_manipulator_manager.h"

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <Eigen/QR>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
  #include <eigen3/Eigen/QR>

  #define PI 3.141592
#endif

namespace robotis_manipulator
{
class MinimumJerk
{
private:
  Eigen::VectorXd coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  void calcCoefficient(Point start,
                       Point goal,
                       double move_time);

  Eigen::VectorXd getCoefficient();
};

class JointTrajectory
{
private:
  uint8_t coefficient_size_;
  MinimumJerk minimum_jerk_trajectory_generator_;
  Eigen::MatrixXd minimum_jerk_coefficient_;

public:
  JointTrajectory();
  virtual ~JointTrajectory();

  void makeJointTrajectory(double move_time,
            JointWaypoint start,
            JointWaypoint goal
            );
  Eigen::MatrixXd getMinimumJerkCoefficient();
  JointWaypoint getJointWaypoint(double tick);
};

class TaskTrajectory
{
private:
  uint8_t coefficient_size_;
  MinimumJerk minimum_jerk_trajectory_generator_;
  Eigen::MatrixXd minimum_jerk_coefficient_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  void makeTaskTrajectory(double move_time,
            TaskWaypoint start,
            TaskWaypoint goal
            );
  Eigen::MatrixXd getMinimumJerkCoefficient();
  TaskWaypoint getTaskWaypoint(double tick);
};


/*****************************************************************************
** Trajectory Class
*****************************************************************************/
class Trajectory
{
private:
  TrajectoryType trajectory_type_;
  Time trajectory_time_;
  Manipulator manipulator_;

  JointTrajectory joint_;
  TaskTrajectory task_;
  std::map<Name, CustomJointTrajectory *> cus_joint_;
  std::map<Name, CustomTaskTrajectory *> cus_task_;

  Name present_custom_trajectory_name_;
  Name present_control_tool_name_;

public:
  Trajectory() {}
  ~Trajectory() {}

  // Time
  void setMoveTime(double move_time);
  void setPresentTime(double present_time);
  void setStartTimeToPresentTime();
  void setStartTime(double start_time);
  double getMoveTime();
  double getTickTime();

  // Manipulator
  void setManipulator(Manipulator manipulator);
  Manipulator* getManipulator();

  // Get Trajectory
  JointTrajectory getJointTrajectory();
  TaskTrajectory getTaskTrajectory();
  CustomJointTrajectory* getCustomJointTrajectory(Name name);
  CustomTaskTrajectory* getCustomTaskTrajectory(Name name);

  // Custom Trajectory Setting
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);
  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  void setPresentControlToolName(Name present_control_tool_name);
  Name getPresentCustomTrajectoryName();
  Name getPresentControlToolName();

  // First Waypoint
  void initTrajectoryWaypoint(Manipulator actual_manipulator, Kinematics *kinematics);

  // Present Waypoint
  void updatePresentWaypoint(Kinematics* kinematics); //forward kinematics,dynamics
  void setPresentJointWaypoint(JointWaypoint joint_value_vector);
  void setPresentTaskWaypoint(Name tool_name, TaskWaypoint tool_position_value_vector);
  JointWaypoint getPresentJointWaypoint();
  TaskWaypoint getPresentTaskWaypoint(Name tool_name);

  JointWaypoint removeWaypointDynamicData(JointWaypoint value);
  TaskWaypoint removeWaypointDynamicData(TaskWaypoint value);

  // Trajectory
  void setTrajectoryType(TrajectoryType trajectory_type);
  bool checkTrajectoryType(TrajectoryType trajectory_type);
  void makeJointTrajectory(JointWaypoint start_way_point, JointWaypoint goal_way_point);
  void makeTaskTrajectory(TaskWaypoint start_way_point, TaskWaypoint goal_way_point);
  void makeCustomTrajectory(Name trajectory_name, JointWaypoint start_way_point, const void *arg);
  void makeCustomTrajectory(Name trajectory_name, TaskWaypoint start_way_point, const void *arg);

  // Tool
  void setToolGoalPosition(Name tool_name, double tool_goal_position);
  void setToolGoalValue(Name tool_name, JointValue tool_goal_value);
  double getToolGoalPosition(Name tool_name);
  JointValue getToolGoalValue(Name tool_name);
};

} // namespace robotis_manipulator
#endif // ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_



