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

namespace ROBOTIS_MANIPULATOR
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
  MinimumJerk trajectory_generator_;
  Eigen::MatrixXd coefficient_;

public:
  JointTrajectory();
  virtual ~JointTrajectory();

  void init(double move_time,
            JointWayPoint start,
            JointWayPoint goal
            );
  Eigen::MatrixXd getCoefficient();
  JointWayPoint getJointWayPoint(double tick);
};

class TaskTrajectory
{
private:
  uint8_t coefficient_size_;
  MinimumJerk trajectory_generator_;
  Eigen::MatrixXd coefficient_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  void init(double move_time,
            TaskWayPoint start,
            TaskWayPoint goal
            );
  Eigen::MatrixXd getCoefficient();
  TaskWayPoint getTaskWayPoint(double tick);
};


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

  //time
  void setMoveTime(double move_time);
  void setPresentTime(double present_time);
  void setStartTimeFromPresentTime();
  void setStartTime(double start_time);
  double getMoveTime();
  double getTickTime();

  //Manipulator
  void setTrajectoryManipulator(Manipulator manipulator);
  Manipulator* getTrajectoryManipulator();

  //get Trajectory
  JointTrajectory getJointTrajectory();
  TaskTrajectory getTaskTrajectory();
  CustomJointTrajectory* getCustomJointTrajectory(Name name);
  CustomTaskTrajectory* getCustomTaskTrajectory(Name name);

  //Custom Trajectory Setting
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);
  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  void setPresentControlToolName(Name present_control_tool_name);
  Name getPresentCustomTrajectoryName();
  Name getPresentControlToolName();

  //First Way Point
  void initTrajectoryWayPoint(Manipulator present_real_manipulator, Kinematics *kinematics);

  //Present Way Point
  void UpdatePresentWayPoint(Kinematics* kinematics); //forward kinematics,dynamics
  void setPresentJointWayPoint(JointWayPoint joint_value_vector);
  void setPresentTaskWayPoint(Name tool_name, TaskWayPoint tool_position_value_vector);
  JointWayPoint getPresentJointWayPoint();
  TaskWayPoint getPresentTaskWayPoint(Name tool_name);

  JointWayPoint removeWayPointDynamicData(JointWayPoint value);
  TaskWayPoint removeWayPointDynamicData(TaskWayPoint value);

  //Trajectory
  void setTrajectoryType(TrajectoryType trajectory_type);
  bool checkTrajectoryType(TrajectoryType trajectory_type);
  void makeJointTrajectory(JointWayPoint start_way_point, JointWayPoint goal_way_point);
  void makeTaskTrajectory(TaskWayPoint start_way_point, TaskWayPoint goal_way_point);
  void makeCustomTrajectory(Name trajectory_name, JointWayPoint start_way_point, const void *arg);
  void makeCustomTrajectory(Name trajectory_name, TaskWayPoint start_way_point, const void *arg);

  //tool
  void setToolGoalPosition(Name tool_name, double tool_goal_position);
  void setToolGoalValue(Name tool_name, JointValue tool_goal_value);
  double getToolGoalPosition(Name tool_name);
  JointValue getToolGoalValue(Name tool_name);
};

} // namespace ROBOTIS_MANIPULATOR
#endif // ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_



