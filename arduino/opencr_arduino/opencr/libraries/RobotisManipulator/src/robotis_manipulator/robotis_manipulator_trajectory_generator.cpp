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

#include "../../include/robotis_manipulator/robotis_manipulator_trajectory_generator.h"

using namespace robotis_manipulator;

MinimumJerk::MinimumJerk()
{
  coefficient_ = Eigen::VectorXd::Zero(6);
}

MinimumJerk::~MinimumJerk() {}

void MinimumJerk::calcCoefficient(Point start,
                                  Point goal,
                                  double move_time)
{
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  A << pow(move_time, 3), pow(move_time, 4), pow(move_time, 5),
      3 * pow(move_time, 2), 4 * pow(move_time, 3), 5 * pow(move_time, 4),
      6 * pow(move_time, 1), 12 * pow(move_time, 2), 20 * pow(move_time, 3);

  coefficient_(0) = start.position;
  coefficient_(1) = start.velocity;
  coefficient_(2) = 0.5 * start.acceleration;

  b << (goal.position - start.position - (start.velocity * move_time + 0.5 * start.acceleration * pow(move_time, 2))),
      (goal.velocity - start.velocity - (start.acceleration * move_time)),
      (goal.acceleration - start.acceleration);

  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> dec(A);
  x = dec.solve(b);

  coefficient_(3) = x(0);
  coefficient_(4) = x(1);
  coefficient_(5) = x(2);
}

Eigen::VectorXd MinimumJerk::getCoefficient()
{
  return coefficient_;
}

//-------------------- Joint trajectory --------------------//

JointTrajectory::JointTrajectory()
{}

JointTrajectory::~JointTrajectory() {}

void JointTrajectory::makeJointTrajectory(double move_time, JointWaypoint start,
                           JointWaypoint goal)
{
  coefficient_size_ = start.size();
  minimum_jerk_coefficient_.resize(6,coefficient_size_);
  for (uint8_t index = 0; index < coefficient_size_; index++)
  {
    minimum_jerk_trajectory_generator_.calcCoefficient(start.at(index),
                                    goal.at(index),
                                    move_time);

    minimum_jerk_coefficient_.col(index) = minimum_jerk_trajectory_generator_.getCoefficient();
  }
}

JointWaypoint JointTrajectory::getJointWaypoint(double tick)
{
  JointWaypoint joint_way_point;
  for (uint8_t index = 0; index < coefficient_size_; index++)
  {
    JointValue single_joint_way_point;

    single_joint_way_point.position = minimum_jerk_coefficient_(0, index) +
             minimum_jerk_coefficient_(1, index) * pow(tick, 1) +
             minimum_jerk_coefficient_(2, index) * pow(tick, 2) +
             minimum_jerk_coefficient_(3, index) * pow(tick, 3) +
             minimum_jerk_coefficient_(4, index) * pow(tick, 4) +
             minimum_jerk_coefficient_(5, index) * pow(tick, 5);

    single_joint_way_point.velocity = minimum_jerk_coefficient_(1, index) +
             2 * minimum_jerk_coefficient_(2, index) * pow(tick, 1) +
             3 * minimum_jerk_coefficient_(3, index) * pow(tick, 2) +
             4 * minimum_jerk_coefficient_(4, index) * pow(tick, 3) +
             5 * minimum_jerk_coefficient_(5, index) * pow(tick, 4);

    single_joint_way_point.acceleration = 2 * minimum_jerk_coefficient_(2, index) +
             6 * minimum_jerk_coefficient_(3, index) * pow(tick, 1) +
             12 * minimum_jerk_coefficient_(4, index) * pow(tick, 2) +
             20 * minimum_jerk_coefficient_(5, index) * pow(tick, 3);

    single_joint_way_point.effort = 0.0;

    joint_way_point.push_back(single_joint_way_point);
  }

  return joint_way_point;
}

Eigen::MatrixXd JointTrajectory::getMinimumJerkCoefficient()
{
  return minimum_jerk_coefficient_;
}

//-------------------- Task trajectory --------------------//

TaskTrajectory::TaskTrajectory()
{
  minimum_jerk_coefficient_ = Eigen::MatrixXd::Identity(6, 4);
}
TaskTrajectory::~TaskTrajectory() {}

void TaskTrajectory::makeTaskTrajectory(double move_time, TaskWaypoint start,
                           TaskWaypoint goal)
{
  std::vector<Point> start_way_point;
  std::vector<Point> goal_way_point;

  ////////////////////////////////////position////////////////////////////////////
  for(uint8_t i = 0; i < 3; i++)      //x, y, z
  {
    Point position_temp;
    position_temp.position = start.kinematic.position[i];
    position_temp.velocity = start.dynamic.linear.velocity[i];
    position_temp.acceleration = start.dynamic.linear.acceleration[i];
    position_temp.effort = 0.0;
    start_way_point.push_back(position_temp);

    position_temp.position = goal.kinematic.position[i];
    position_temp.velocity = goal.dynamic.linear.velocity[i];
    position_temp.acceleration = goal.dynamic.linear.acceleration[i];
    position_temp.effort = 0.0;
    goal_way_point.push_back(position_temp);
  }
  ////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////orientation///////////////////////////////////

  Eigen::Vector3d start_orientation_rpy;
  Eigen::Vector3d start_ang_vel_rpy;
  Eigen::Vector3d start_ang_acc_rpy;

  start_orientation_rpy = math::convertRotationMatrixToRPYVector(start.kinematic.orientation);
  start_ang_vel_rpy = math::convertOmegaToRPYVelocity(start_orientation_rpy, start.dynamic.angular.velocity);
  start_ang_acc_rpy = math::convertOmegaDotToRPYAcceleration(start_orientation_rpy, start_ang_vel_rpy, start.dynamic.angular.acceleration);

  Eigen::Vector3d goal_orientation_rpy;
  Eigen::Vector3d goal_ang_vel_rpy;
  Eigen::Vector3d goal_ang_acc_rpy;

  goal_orientation_rpy = math::convertRotationMatrixToRPYVector(goal.kinematic.orientation);
  goal_ang_vel_rpy = math::convertOmegaToRPYVelocity(goal_orientation_rpy, goal.dynamic.angular.velocity);
  start_ang_acc_rpy = math::convertOmegaDotToRPYAcceleration(goal_orientation_rpy, goal_ang_vel_rpy, goal.dynamic.angular.acceleration);

  for(uint8_t i = 0; i < 3; i++)    //roll, pitch, yaw
  {
    Point orientation_temp;
    orientation_temp.position = start_orientation_rpy[i];
    orientation_temp.velocity = start_ang_vel_rpy[i];
    orientation_temp.acceleration = start_ang_acc_rpy[i];
    orientation_temp.effort = 0.0;
    start_way_point.push_back(orientation_temp);

    orientation_temp.position = goal_orientation_rpy[i];
    orientation_temp.velocity = goal_ang_vel_rpy[i];
    orientation_temp.acceleration = goal_ang_acc_rpy[i];
    orientation_temp.effort = 0.0;
    goal_way_point.push_back(orientation_temp);
  }
  ////////////////////////////////////////////////////////////////////////////////

  coefficient_size_ = start_way_point.size();
  minimum_jerk_coefficient_.resize(6,coefficient_size_);
  for (uint8_t index = 0; index < coefficient_size_; index++)
  {
    minimum_jerk_trajectory_generator_.calcCoefficient(start_way_point.at(index),
                                    goal_way_point.at(index),
                                    move_time);

    minimum_jerk_coefficient_.col(index) = minimum_jerk_trajectory_generator_.getCoefficient();
  }
}

TaskWaypoint TaskTrajectory::getTaskWaypoint(double tick)
{
  std::vector<Point> result_point;
  for (uint8_t index = 0; index < coefficient_size_; index++)
  {
    Point single_task_way_point;

    single_task_way_point.position = minimum_jerk_coefficient_(0, index) +
             minimum_jerk_coefficient_(1, index) * pow(tick, 1) +
             minimum_jerk_coefficient_(2, index) * pow(tick, 2) +
             minimum_jerk_coefficient_(3, index) * pow(tick, 3) +
             minimum_jerk_coefficient_(4, index) * pow(tick, 4) +
             minimum_jerk_coefficient_(5, index) * pow(tick, 5);

    single_task_way_point.velocity = minimum_jerk_coefficient_(1, index) +
             2 * minimum_jerk_coefficient_(2, index) * pow(tick, 1) +
             3 * minimum_jerk_coefficient_(3, index) * pow(tick, 2) +
             4 * minimum_jerk_coefficient_(4, index) * pow(tick, 3) +
             5 * minimum_jerk_coefficient_(5, index) * pow(tick, 4);

    single_task_way_point.acceleration = 2 * minimum_jerk_coefficient_(2, index) +
             6 * minimum_jerk_coefficient_(3, index) * pow(tick, 1) +
             12 * minimum_jerk_coefficient_(4, index) * pow(tick, 2) +
             20 * minimum_jerk_coefficient_(5, index) * pow(tick, 3);

    single_task_way_point.effort = 0.0;

    result_point.push_back(single_task_way_point);
  }

  TaskWaypoint task_way_point;
  ////////////////////////////////////position////////////////////////////////////
  for(uint8_t i = 0; i < 3; i++)        //x ,y ,z
  {
    task_way_point.kinematic.position[i] = result_point.at(i).position;
    task_way_point.dynamic.linear.velocity[i] = result_point.at(i).velocity;
    task_way_point.dynamic.linear.acceleration[i] = result_point.at(i).acceleration;
  }
  ////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////orientation///////////////////////////////////
  Eigen::Vector3d rpy_orientation;
  rpy_orientation << result_point.at(3).position, result_point.at(4).position, result_point.at(5).position;
  task_way_point.kinematic.orientation = math::convertRPYToRotationMatrix(result_point.at(3).position,   //roll
                                                                       result_point.at(4).position,   //pitch
                                                                       result_point.at(5).position);   //yaw

  Eigen::Vector3d rpy_velocity;
  rpy_velocity << result_point.at(3).velocity, result_point.at(4).velocity, result_point.at(5).velocity;
  task_way_point.dynamic.angular.velocity = math::convertRPYVelocityToOmega(rpy_orientation, rpy_velocity);

  Eigen::Vector3d rpy_acceleration;
  rpy_acceleration << result_point.at(3).acceleration, result_point.at(4).acceleration, result_point.at(5).acceleration;
  task_way_point.dynamic.angular.acceleration = math::convertRPYAccelerationToOmegaDot(rpy_orientation, rpy_velocity, rpy_acceleration);

  return task_way_point;
}

Eigen::MatrixXd TaskTrajectory::getMinimumJerkCoefficient()
{
  return minimum_jerk_coefficient_;
}


/*****************************************************************************
** Trajectory Class
*****************************************************************************/
void Trajectory::setMoveTime(double move_time)
{
  trajectory_time_.total_move_time = move_time;
}

void Trajectory::setPresentTime(double present_time)
{
  trajectory_time_.present_time = present_time;
}

void Trajectory::setStartTimeToPresentTime()
{
  trajectory_time_.start_time = trajectory_time_.present_time;
}

void Trajectory::setStartTime(double start_time)
{
  trajectory_time_.start_time = start_time;
}

double Trajectory::getMoveTime()
{
  return trajectory_time_.total_move_time;
}

double Trajectory::getTickTime()
{
  return trajectory_time_.present_time - trajectory_time_.start_time;
}

void Trajectory::setManipulator(Manipulator manipulator)
{
  manipulator_= manipulator;
}

Manipulator* Trajectory::getManipulator()
{
  return &manipulator_;
}

JointTrajectory Trajectory::getJointTrajectory()
{
  return joint_;
}

TaskTrajectory Trajectory::getTaskTrajectory()
{
  return task_;
}

CustomJointTrajectory *Trajectory::getCustomJointTrajectory(Name name)
{
  return cus_joint_.at(name);
}

CustomTaskTrajectory *Trajectory::getCustomTaskTrajectory(Name name)
{
  return cus_task_.at(name);
}

void Trajectory::addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory)
{
  cus_joint_.insert(std::make_pair(trajectory_name, custom_trajectory));
}

void Trajectory::addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory)
{
  cus_task_.insert(std::make_pair(trajectory_name, custom_trajectory));
}

void Trajectory::setCustomTrajectoryOption(Name trajectory_name, const void* arg)
{
  if(cus_joint_.find(trajectory_name) != cus_joint_.end())
    cus_joint_.at(trajectory_name)->setOption(arg);
  else if(cus_task_.find(trajectory_name) != cus_task_.end())
    cus_task_.at(trajectory_name)->setOption(arg);
}

void Trajectory::setPresentControlToolName(Name present_control_tool_name)
{
  present_control_tool_name_ = present_control_tool_name;
}

Name Trajectory::getPresentCustomTrajectoryName()
{
  return present_custom_trajectory_name_;
}

Name Trajectory::getPresentControlToolName()
{
 return present_control_tool_name_;
}

void Trajectory::initTrajectoryWaypoint(Manipulator actual_manipulator, Kinematics *kinematics)
{
  setManipulator(actual_manipulator);
  JointWaypoint joint_way_point_vector;
  joint_way_point_vector = getManipulator()->getAllActiveJointValue();

  setPresentJointWaypoint(joint_way_point_vector);
  updatePresentWaypoint(kinematics);
}

void Trajectory::updatePresentWaypoint(Kinematics *kinematics)
{
  //kinematics
  kinematics->solveForwardKinematics(&manipulator_);
}

void Trajectory::setPresentJointWaypoint(JointWaypoint joint_value_vector)
{
  manipulator_.setAllActiveJointValue(joint_value_vector);
}

void Trajectory::setPresentTaskWaypoint(Name tool_name, TaskWaypoint tool_value_vector)
{
  manipulator_.setComponentPoseFromWorld(tool_name, tool_value_vector);
}

JointWaypoint Trajectory::getPresentJointWaypoint()
{
  return manipulator_.getAllActiveJointValue();
}

TaskWaypoint Trajectory::getPresentTaskWaypoint(Name tool_name)
{
  return manipulator_.getComponentPoseFromWorld(tool_name);
}

JointWaypoint Trajectory::removeWaypointDynamicData(JointWaypoint value)
{
  for(uint32_t index =0; index < value.size(); index++)
  {
    value.at(index).velocity = 0.0;
    value.at(index).acceleration = 0.0;
    value.at(index).effort = 0.0;
  }
  return value;
}

TaskWaypoint Trajectory::removeWaypointDynamicData(TaskWaypoint value)
{
  value.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  value.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  value.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  value.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);
  return value;
}


//Trajectory
void Trajectory::setTrajectoryType(TrajectoryType trajectory_type)
{
  trajectory_type_ = trajectory_type;
}

bool Trajectory::checkTrajectoryType(TrajectoryType trajectory_type)
{
  if(trajectory_type_==trajectory_type)
    return true;
  else
    return false;
}

void Trajectory::makeJointTrajectory(JointWaypoint start_way_point, JointWaypoint goal_way_point)
{
  joint_.makeJointTrajectory(trajectory_time_.total_move_time, start_way_point, goal_way_point);
}

void Trajectory::makeTaskTrajectory(TaskWaypoint start_way_point, TaskWaypoint goal_way_point)
{
  task_.makeTaskTrajectory(trajectory_time_.total_move_time, start_way_point, goal_way_point);
}

void Trajectory::makeCustomTrajectory(Name trajectory_name, JointWaypoint start_way_point, const void *arg)
{
  if(cus_joint_.find(trajectory_name) != cus_joint_.end())
  {
    present_custom_trajectory_name_ = trajectory_name;
    cus_joint_.at(trajectory_name)->makeJointTrajectory(trajectory_time_.total_move_time, start_way_point, arg);
  }
  else
    log::error("[makeCustomTrajectory] Wrong way point type.");
}

void Trajectory::makeCustomTrajectory(Name trajectory_name, TaskWaypoint start_way_point, const void *arg)
{
  if(cus_task_.find(trajectory_name) != cus_task_.end())
  {
    present_custom_trajectory_name_ = trajectory_name;
    cus_task_.at(trajectory_name)->makeTaskTrajectory(trajectory_time_.total_move_time, start_way_point, arg);
  }
  else
    log::error("[makeCustomTrajectory] Wrong way point type.");
}

//tool
void Trajectory::setToolGoalPosition(Name tool_name, double tool_goal_position)
{
  manipulator_.setJointPosition(tool_name, tool_goal_position);
}


void Trajectory::setToolGoalValue(Name tool_name, JointValue tool_goal_value)
{
  manipulator_.setJointValue(tool_name, tool_goal_value);
}

double Trajectory::getToolGoalPosition(Name tool_name)
{
  return manipulator_.getJointPosition(tool_name);
}

JointValue Trajectory::getToolGoalValue(Name tool_name)
{
  return manipulator_.getJointValue(tool_name);
}
