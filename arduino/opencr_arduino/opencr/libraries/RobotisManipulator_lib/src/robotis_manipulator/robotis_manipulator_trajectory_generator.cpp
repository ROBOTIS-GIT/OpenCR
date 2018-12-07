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

using namespace ROBOTIS_MANIPULATOR;


MinimumJerk::MinimumJerk()
{
  coefficient_ = Eigen::VectorXd::Zero(6);
}

MinimumJerk::~MinimumJerk() {}

void MinimumJerk::calcCoefficient(WayPoint start,
                                  WayPoint goal,
                                  double move_time,
                                  double control_time)
{
  uint16_t step_time = uint16_t(floor(move_time / control_time) + 1.0);
  move_time = double(step_time - 1) * control_time;

  Eigen::Matrix3d A = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  A << pow(move_time, 3), pow(move_time, 4), pow(move_time, 5),
      3 * pow(move_time, 2), 4 * pow(move_time, 3), 5 * pow(move_time, 4),
      6 * pow(move_time, 1), 12 * pow(move_time, 2), 20 * pow(move_time, 3);

  coefficient_(0) = start.value;
  coefficient_(1) = start.velocity;
  coefficient_(2) = 0.5 * start.acceleration;

  b << (goal.value - start.value - (start.velocity * move_time + 0.5 * start.acceleration * pow(move_time, 2))),
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

void JointTrajectory::init(double move_time,
                           double control_time, std::vector<WayPoint> start,
                           std::vector<WayPoint> goal)
{
  for (uint8_t index = 0; index < start.size(); index++)
  {
    trajectory_generator_.calcCoefficient(start.at(index),
                                    goal.at(index),
                                    move_time,
                                    control_time);

    coefficient_.col(index) = trajectory_generator_.getCoefficient();
  }
}

void JointTrajectory::setJointNum(uint8_t joint_num)
{
  joint_num_ = joint_num;
  coefficient_ = Eigen::MatrixXd::Identity(6, joint_num);
}

std::vector<WayPoint> JointTrajectory::getJointWayPoint(double tick)
{
  joint_way_point_.clear();
  for (uint8_t index = 0; index < joint_num_; index++)
  {
    WayPoint single_joint_way_point;
    single_joint_way_point.value = 0.0;
    single_joint_way_point.velocity = 0.0;
    single_joint_way_point.acceleration = 0.0;
    single_joint_way_point.effort = 0.0;

    single_joint_way_point.value = coefficient_(0, index) +
             coefficient_(1, index) * pow(tick, 1) +
             coefficient_(2, index) * pow(tick, 2) +
             coefficient_(3, index) * pow(tick, 3) +
             coefficient_(4, index) * pow(tick, 4) +
             coefficient_(5, index) * pow(tick, 5);

    single_joint_way_point.velocity = coefficient_(1, index) +
             2 * coefficient_(2, index) * pow(tick, 1) +
             3 * coefficient_(3, index) * pow(tick, 2) +
             4 * coefficient_(4, index) * pow(tick, 3) +
             5 * coefficient_(5, index) * pow(tick, 4);

    single_joint_way_point.acceleration = 2 * coefficient_(2, index) +
             6 * coefficient_(3, index) * pow(tick, 1) +
             12 * coefficient_(4, index) * pow(tick, 2) +
             20 * coefficient_(5, index) * pow(tick, 3);

    joint_way_point_.push_back(single_joint_way_point);
  }

  return joint_way_point_;
}

Eigen::MatrixXd JointTrajectory::getCoefficient()
{
  return coefficient_;
}

//-------------------- Task trajectory --------------------//

TaskTrajectory::TaskTrajectory()
{
  dof_ = 6;
  position_coefficient_ = Eigen::MatrixXd::Identity(6, dof_);
}
TaskTrajectory::~TaskTrajectory() {}

void TaskTrajectory::init(double move_time,
                           double control_time, std::vector<WayPoint> start,
                           std::vector<WayPoint> goal)
{
  for (uint8_t index = 0; index < start.size(); index++)
  {
    trajectory_generator_.calcCoefficient(start.at(index),
                                    goal.at(index),
                                    move_time,
                                    control_time);

    position_coefficient_.col(index) = trajectory_generator_.getCoefficient();
  }
}

std::vector<WayPoint> TaskTrajectory::getTaskWayPoint(double tick)
{
  task_position_way_point_.clear();
  for (uint8_t index = 0; index < dof_; index++)
  {
    WayPoint single_task_position_way_point;
    single_task_position_way_point.value = 0.0;
    single_task_position_way_point.velocity = 0.0;
    single_task_position_way_point.effort = 0.0;

    single_task_position_way_point.value = position_coefficient_(0, index) +
             position_coefficient_(1, index) * pow(tick, 1) +
             position_coefficient_(2, index) * pow(tick, 2) +
             position_coefficient_(3, index) * pow(tick, 3) +
             position_coefficient_(4, index) * pow(tick, 4) +
             position_coefficient_(5, index) * pow(tick, 5);

    single_task_position_way_point.velocity = position_coefficient_(1, index) +
             2 * position_coefficient_(2, index) * pow(tick, 1) +
             3 * position_coefficient_(3, index) * pow(tick, 2) +
             4 * position_coefficient_(4, index) * pow(tick, 3) +
             5 * position_coefficient_(5, index) * pow(tick, 4);

    single_task_position_way_point.effort = 2 * position_coefficient_(2, index) +
             6 * position_coefficient_(3, index) * pow(tick, 1) +
             12 * position_coefficient_(4, index) * pow(tick, 2) +
             20 * position_coefficient_(5, index) * pow(tick, 3);

    task_position_way_point_.push_back(single_task_position_way_point);
  }

  return task_position_way_point_;
}


Eigen::MatrixXd TaskTrajectory::getCoefficient()
{
  return position_coefficient_;
}



//------------------------ trajectory ------------------------//

void Trajectory::setMoveTime(double move_time)
{
  trajectory_time_.total_move_time = move_time;
}

void Trajectory::setPresentTime(double present_time)
{
  trajectory_time_.present_time = present_time;
}

void Trajectory::setStartTimeFromPresentTime()
{
  trajectory_time_.start_time = trajectory_time_.present_time;
}

void Trajectory::setControlLoopTime(double control_time)
{
  trajectory_time_.control_loop_time = control_time;
}

double Trajectory::getMoveTime()
{
  return trajectory_time_.total_move_time;
}

double Trajectory::getControlLoopTime()
{
  return trajectory_time_.control_loop_time;
}

double Trajectory::getTickTime()
{
  return trajectory_time_.present_time - trajectory_time_.start_time;
}

void Trajectory::setTrajectoryManipulator(Manipulator manipulator)
{
  manipulator_= manipulator;
}

Manipulator* Trajectory::getTrajectoryManipulator()
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

void Trajectory::addDrawingTrajectory(Name name, DrawingTrajectory *drawing)
{
  drawing_.insert(std::make_pair(name, drawing));
}

DrawingTrajectory* Trajectory::getDrawingtrajectory(Name name)
{
  return drawing_.at(name);
}

void Trajectory::setDrawingOption(Name name, const void* arg)
{
  drawing_.at(name)->setOption(arg);
}


void Trajectory::setPresentDrawingObjectName(Name present_drawing_object_name)
{
  present_drawing_object_name_ = present_drawing_object_name;
}

void Trajectory::setPresentControlToolName(Name present_control_tool_name)
{
  present_control_tool_name_ = present_control_tool_name;
}

Name Trajectory::getPresentDrawingObjectName()
{
  return present_drawing_object_name_;
}

Name Trajectory::getPresentControlToolName()
{
 return present_control_tool_name_;
}

void Trajectory::initTrajectoryWayPoint(double present_time, Manipulator present_real_manipulator, Kinematics* kinematics)
{
  setTrajectoryManipulator(present_real_manipulator);
  std::vector<WayPoint> joint_way_point_vector;
  std::vector<double> joint_value_vector;
  joint_value_vector = getTrajectoryManipulator()->getAllActiveJointValue();

  for(uint32_t index=0; index < joint_value_vector.size(); index++)
  {
    WayPoint joint_way_point;
    joint_way_point.value = joint_value_vector.at(index);
    joint_way_point.velocity = 0.0;
    joint_way_point.acceleration = 0.0;
    joint_way_point.effort = 0.0;

    joint_way_point_vector.push_back(joint_way_point);
  }

  setPresentJointWayPoint(joint_way_point_vector);
  UpdatePresentWayPoint(kinematics);
  setPresentTime(present_time);
}

void Trajectory::UpdatePresentWayPoint(Kinematics* kinematics)
{
  //kinematics (position)
  kinematics->updatePassiveJointValue(&manipulator_);
  kinematics->forwardKinematics(&manipulator_);

  //dynamics (velocity)
  std::map<Name, Component>::iterator it;
  Eigen::VectorXd joint_velocity(manipulator_.getDOF());

  Eigen::VectorXd pose_velocity(6);
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  int8_t index = 0;
  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.checkComponentType(it->first, ACTIVE_JOINT_COMPONENT)) // Check whether Active or Passive
    {
      // Active
      joint_velocity[index] = manipulator_.getVelocity(it->first);
      index++;
    }
  }

  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    pose_velocity = kinematics->jacobian(&manipulator_, it->first)*joint_velocity;
    linear_velocity[0] = pose_velocity[0];
    linear_velocity[1] = pose_velocity[1];
    linear_velocity[2] = pose_velocity[2];
    angular_velocity[0] = pose_velocity[3];
    angular_velocity[1] = pose_velocity[4];
    angular_velocity[2] = pose_velocity[5];
    Dynamicpose dynamic_pose;
    dynamic_pose.linear.velocity = linear_velocity;
    dynamic_pose.angular.velocity = angular_velocity;
    dynamic_pose.linear.acceleration = Eigen::Vector3d::Zero();
    dynamic_pose.angular.acceleration = Eigen::Vector3d::Zero();

    manipulator_.setComponentDynamicPoseFromWorld(it->first, dynamic_pose);
  }
}

void Trajectory::setPresentJointWayPoint(std::vector<WayPoint> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.checkComponentType(it->first, ACTIVE_JOINT_COMPONENT))
    {
      manipulator_.setValue(it->first, joint_value_vector.at(index).value);
      manipulator_.setVelocity(it->first, joint_value_vector.at(index).velocity);
      manipulator_.setAcceleration(it->first, joint_value_vector.at(index).acceleration);
      manipulator_.setEffort(it->first, joint_value_vector.at(index).effort);

      index++;
    }
  }
}

void Trajectory::setPresentTaskWayPoint(Name tool_name, std::vector<WayPoint> tool_value_vector)
{
  Pose pose_to_world;
  Dynamicpose dynamic_pose;
  for(int pos_count = 0; pos_count < 3; pos_count++)
  {
    pose_to_world.position[pos_count] = tool_value_vector.at(pos_count).value;
    dynamic_pose.linear.velocity[pos_count] = tool_value_vector.at(pos_count).velocity;
    dynamic_pose.linear.acceleration[pos_count] = tool_value_vector.at(pos_count).acceleration;
  }

  Eigen::Vector3d orientation_value_vector;
  Eigen::Vector3d orientation_velocity_vector;
  Eigen::Vector3d orientation_acceleration_vector;
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    orientation_value_vector[ori_count] = tool_value_vector.at(ori_count+3).value;
    orientation_velocity_vector[ori_count] = tool_value_vector.at(ori_count+3).velocity;
    orientation_acceleration_vector[ori_count] = tool_value_vector.at(ori_count+3).acceleration;
  }
  Eigen::Matrix3d orientation;
  orientation = RM_MATH::convertRPYToRotation(orientation_value_vector[0], orientation_value_vector[1], orientation_value_vector[2]);
  pose_to_world.orientation = orientation;
  dynamic_pose.angular.velocity = orientation_velocity_vector;
  dynamic_pose.angular.acceleration = orientation_acceleration_vector;

  manipulator_.setComponentPoseFromWorld(tool_name, pose_to_world);
  manipulator_.setComponentDynamicPoseFromWorld(tool_name, dynamic_pose);
}

std::vector<WayPoint> Trajectory::getPresentJointWayPoint()
{
  std::map<Name, Component>::iterator it;
  WayPoint result;
  std::vector<WayPoint> result_vector;

  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.checkComponentType(it->first, ACTIVE_JOINT_COMPONENT)) // Check whether Active or Passive
    {
      // Active
      result.value = manipulator_.getValue(it->first);
      result.velocity = manipulator_.getVelocity(it->first);
      result.acceleration = manipulator_.getAcceleration(it->first);
      result.effort = manipulator_.getEffort(it->first);
      result_vector.push_back(result);
    }
  }
  return result_vector;
}

std::vector<WayPoint> Trajectory::getPresentTaskWayPoint(Name tool_name)
{
  std::vector<WayPoint> result_vector;
  WayPoint result;
  for(int pos_count = 0; pos_count < 3; pos_count++)
  {
    result.value = manipulator_.getComponentPoseFromWorld(tool_name).position[pos_count];
    result.velocity = manipulator_.getComponentDynamicPoseFromWorld(tool_name).linear.velocity[pos_count];
    result.acceleration = manipulator_.getComponentDynamicPoseFromWorld(tool_name).linear.acceleration[pos_count];
    result.effort = 0.0;
    result_vector.push_back(result);
  }

  Eigen::Vector3d orientation_vector =  RM_MATH::convertRotationToRPY(manipulator_.getComponentPoseFromWorld(tool_name).orientation);
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    result.value = orientation_vector[ori_count];
    result.velocity = manipulator_.getComponentDynamicPoseFromWorld(tool_name).angular.velocity[ori_count];
    result.acceleration = manipulator_.getComponentDynamicPoseFromWorld(tool_name).angular.acceleration[ori_count];
    result.effort = 0.0;
    result_vector.push_back(result);
  }

  return result_vector;
}

void Trajectory::setStartWayPoint(std::vector<WayPoint> start_way_point)
{
  start_way_point_ = start_way_point;
}

void Trajectory::setGoalWayPoint(std::vector<WayPoint> goal_way_point)
{
  goal_way_point_ = goal_way_point;
}

void Trajectory::clearStartWayPoint()
{
  start_way_point_.clear();
}

void Trajectory::clearGoalWayPoint()
{
  goal_way_point_.clear();
}

std::vector<WayPoint> Trajectory::getStartWayPoint()
{
  return start_way_point_;
}

std::vector<WayPoint> Trajectory::getGoalWayPoint()
{
  return goal_way_point_;
}

std::vector<WayPoint> Trajectory::removeWayPointDynamicData(std::vector<WayPoint> value)
{
  for(uint32_t index =0; index < value.size(); index++)
  {
    value.at(index).velocity = 0.0;
    value.at(index).acceleration = 0.0;
    value.at(index).effort = 0.0;
  }
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

void Trajectory::makeJointTrajectory()
{
  joint_.setJointNum(manipulator_.getDOF());
  joint_.init(trajectory_time_.total_move_time, trajectory_time_.control_loop_time, start_way_point_, goal_way_point_);
}

void Trajectory::makeTaskTrajectory()
{
  task_.init(trajectory_time_.total_move_time, trajectory_time_.control_loop_time, start_way_point_, goal_way_point_);
}

void Trajectory::makeDrawingTrajectory(Name drawing_name, const void *arg)
{
  drawing_.at(drawing_name)->init(trajectory_time_.total_move_time, trajectory_time_.control_loop_time, start_way_point_, arg);
}


//tool
void Trajectory::setToolGoalValue(Name name, double tool_goal_value)
{
  manipulator_.setValue(name, tool_goal_value);
}

double Trajectory::getToolGoalValue(Name name)
{
  return manipulator_.getValue(name);
}






























