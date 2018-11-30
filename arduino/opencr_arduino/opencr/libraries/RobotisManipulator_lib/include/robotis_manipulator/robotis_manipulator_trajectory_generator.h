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

  void calcCoefficient(WayPoint start,
                       WayPoint goal,
                       double move_time,
                       double control_time);

  Eigen::VectorXd getCoefficient();
};

class JointTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t joint_num_;
  Eigen::MatrixXd coefficient_;
  std::vector<WayPoint> joint_way_point_;

public:
  JointTrajectory();
  virtual ~JointTrajectory();

  void setJointNum(uint8_t joint_num);
  void init(double move_time,
            double control_time,
            std::vector<WayPoint> start,
            std::vector<WayPoint> goal
            );

  std::vector<WayPoint> getJointWayPoint(double tick);

  Eigen::MatrixXd getCoefficient();
};

class TaskTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t dof_;
  Eigen::MatrixXd position_coefficient_;
  std::vector<WayPoint> task_position_way_point_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  void init(double move_time,
            double control_time,
            std::vector<WayPoint> start,
            std::vector<WayPoint> goal
            );
  std::vector<WayPoint> getTaskWayPoint(double tick);
  Eigen::MatrixXd getCoefficient();
};


class Trajectory
{
private:
  TrajectoryType trajectory_type_;
  Manipulator manipulator_;

  std::vector<WayPoint> start_way_point_;
  std::vector<WayPoint> goal_way_point_;

  JointTrajectory joint_;
  TaskTrajectory task_;
  std::map<Name, DrawingTrajectory *> drawing_;
  Name present_drawing_object_name_;
  Name present_control_tool_name_;

  Time trajectory_time_;

public:
  Trajectory() {}
  ~Trajectory() {}

  //time
  void setMoveTime(double move_time);
  void setPresentTime(double present_time);
  void setStartTimeFromPresentTime();
  void setControlLoopTime(double control_time);
  double getMoveTime();
  double getControlLoopTime();
  double getTickTime();

  //Manipulator
  void setTrajectoryManipulator(Manipulator manipulator);
  Manipulator* getTrajectoryManipulator();

  //Joint
  JointTrajectory getJointTrajectory();

  //Task
  TaskTrajectory getTaskTrajectory();

  //Drawing
  void addDrawingTrajectory(Name name, DrawingTrajectory *drawing);
  DrawingTrajectory* getDrawingtrajectory(Name name);
  void setDrawingOption(Name name, const void* arg);
  void setPresentDrawingObjectName(Name present_drawing_object_name);
  void setPresentControlToolName(Name present_control_tool_name);
  Name getPresentDrawingObjectName();
  Name getPresentControlToolName();

  //Way Point
  void initTrajectoryWayPoint(double present_time, Manipulator present_real_manipulator, Kinematics *kinematics);
  void UpdatePresentWayPoint(Kinematics* kinematics); //forward kinematics,dynamics
  void setPresentJointWayPoint(std::vector<WayPoint> joint_value_vector);
  void setPresentTaskWayPoint(Name tool_name, std::vector<WayPoint> tool_position_value_vector);
  std::vector<WayPoint> getPresentJointWayPoint();
  std::vector<WayPoint> getPresentTaskWayPoint(Name tool_name);

  void setStartWayPoint(std::vector<WayPoint> start_way_point);
  void setGoalWayPoint(std::vector<WayPoint> goal_way_point);
  void clearStartWayPoint();
  void clearGoalWayPoint();
  std::vector<WayPoint> getStartWayPoint();
  std::vector<WayPoint> getGoalWayPoint();

  std::vector<WayPoint> removeWayPointDynamicData(std::vector<WayPoint> value);

  //Trajectory
  void setTrajectoryType(TrajectoryType trajectory_type);
  bool checkTrajectoryType(TrajectoryType trajectory_type);
  void makeJointTrajectory();
  void makeTaskTrajectory();
  void makeDrawingTrajectory(Name drawing_name, const void *arg);

  //tool
  void setToolGoalValue(Name name, double tool_goal_value);
  double getToolGoalValue(Name name);
};


} // namespace RM_TRAJECTORY
#endif // RMTRAJECTORY_H_




