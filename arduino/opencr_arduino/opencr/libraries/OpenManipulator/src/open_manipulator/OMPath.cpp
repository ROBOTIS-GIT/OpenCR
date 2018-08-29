/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM */

#include "../../include/open_manipulator/OMPath.h"

using namespace OM_PATH;
using namespace Eigen;

MinimumJerk::MinimumJerk()
{
  coefficient_ = VectorXf::Zero(6);
}

MinimumJerk::~MinimumJerk() {}

void MinimumJerk::calcCoefficient(Trajectory start,
                                  Trajectory goal,
                                  float move_time,
                                  float control_time)
{
  uint16_t step_time = uint16_t(floor(move_time / control_time) + 1.0);
  move_time = float(step_time - 1) * control_time;

  Matrix3f A = Matrix3f::Identity(3, 3);
  Vector3f x = Vector3f::Zero();
  Vector3f b = Vector3f::Zero();

  A << pow(move_time, 3), pow(move_time, 4), pow(move_time, 5),
      3 * pow(move_time, 2), 4 * pow(move_time, 3), 5 * pow(move_time, 4),
      6 * pow(move_time, 1), 12 * pow(move_time, 2), 20 * pow(move_time, 3);

  coefficient_(0) = start.position;
  coefficient_(1) = start.velocity;
  coefficient_(2) = 0.5 * start.acceleration;

  b << (goal.position - start.position - (start.velocity * move_time + 0.5 * start.acceleration * pow(move_time, 2))),
      (goal.velocity - start.velocity - (start.acceleration * move_time)),
      (goal.acceleration - start.acceleration);

  ColPivHouseholderQR<Matrix3f> dec(A);
  x = dec.solve(b);

  coefficient_(3) = x(0);
  coefficient_(4) = x(1);
  coefficient_(5) = x(2);
}

VectorXf MinimumJerk::getCoefficient()
{
  return coefficient_;
}

JointTrajectory::JointTrajectory(uint8_t joint_num)
{
  joint_num_ = joint_num;
  coefficient_ = MatrixXf::Identity(6, joint_num);
  position_.reserve(joint_num);
  velocity_.reserve(joint_num);
  acceleration_.reserve(joint_num);
}

JointTrajectory::~JointTrajectory() {}

void JointTrajectory::init(std::vector<Trajectory> start,
                           std::vector<Trajectory> goal,
                           float move_time,
                           float control_time)
{
  for (uint8_t index = 0; index < start.size(); index++)
  {
    path_generator_.calcCoefficient(start.at(index),
                                    goal.at(index),
                                    move_time,
                                    control_time);

    coefficient_.col(index) = path_generator_.getCoefficient();
  }
}

std::vector<float> JointTrajectory::getPosition(float tick)
{
  position_.clear();
  for (uint8_t index = 0; index < joint_num_; index++)
  {
    float result = 0.0;
    result = coefficient_(0, index) +
             coefficient_(1, index) * pow(tick, 1) +
             coefficient_(2, index) * pow(tick, 2) +
             coefficient_(3, index) * pow(tick, 3) +
             coefficient_(4, index) * pow(tick, 4) +
             coefficient_(5, index) * pow(tick, 5);

    position_.push_back(result);
  }

  return position_;
}

std::vector<float> JointTrajectory::getVelocity(float tick)
{
  velocity_.clear();
  for (uint8_t index = 0; index < joint_num_; index++)
  {
    float result = 0.0;
    result = coefficient_(1, index) +
             2 * coefficient_(2, index) * pow(tick, 1) +
             3 * coefficient_(3, index) * pow(tick, 2) +
             4 * coefficient_(4, index) * pow(tick, 3) +
             5 * coefficient_(5, index) * pow(tick, 4);

    velocity_.push_back(result);
  }

  return velocity_;
}

std::vector<float> JointTrajectory::getAcceleration(float tick)
{
  acceleration_.clear();
  for (uint8_t index = 0; index < joint_num_; index++)
  {
    float result = 0.0;

    result = 2 * coefficient_(2, index) +
             6 * coefficient_(3, index) * pow(tick, 1) +
             12 * coefficient_(4, index) * pow(tick, 2) +
             20 * coefficient_(5, index) * pow(tick, 3);
          
    acceleration_.push_back(result);
  }

  return acceleration_;
}

MatrixXf JointTrajectory::getCoefficient()
{
  return coefficient_;
}

Line::Line(uint8_t joint_num)
{
  joint_num_ = joint_num;
  coefficient_ = MatrixXf::Identity(6, joint_num);
}

Line::~Line() {}

void Line::init(float move_time, float control_time)
{
  Trajectory start;
  Trajectory goal;

  start.position = 0.0;
  start.velocity = 0.0;
  start.acceleration = 0.0;

  goal.position = 2*M_PI;
  goal.velocity = 0.0;
  goal.acceleration = 0.0;

  path_generator_.calcCoefficient(start,
                                  goal,
                                  move_time,
                                  control_time);

  coefficient_ = path_generator_.getCoefficient();
}

void Line::setTwoPoints(Vector3f start, Vector3f end)
{
  start_ = start;
  end_ = end;
}

Pose Line::getPose(float tick)
{
  Pose pose;
  float get_time_var = 0.0;

  get_time_var = coefficient_(0) +
             coefficient_(1) * pow(tick, 1) +
             coefficient_(2) * pow(tick, 2) +
             coefficient_(3) * pow(tick, 3) +
             coefficient_(4) * pow(tick, 4) +
             coefficient_(5) * pow(tick, 5);

  // Get direction of parametric equation
  Vector3f start_to_end;
  start_to_end(0) = end_(0) - start_(0);
  start_to_end(1) = end_(1) - start_(1);
  start_to_end(2) = end_(2) - start_(2);

  // Get parametric equation
  pose.position(0) = start_(0) + (start_to_end(0) * get_time_var);
  pose.position(1) = start_(1) + (start_to_end(1) * get_time_var);
  pose.position(2) = start_(2) + (start_to_end(2) * get_time_var);

  return pose;
}

Circle::Circle(uint8_t joint_num)
{
  joint_num_ = joint_num;
  coefficient_ = MatrixXf::Identity(6, joint_num);
}

Circle::~Circle() {}

void Circle::init(Vector3f initial_position, float radius, float move_time, float control_time)
{
  initial_position_ = initial_position;
  radius_ = radius;

  Trajectory start;
  Trajectory goal;

  start.position = 0.0;
  start.velocity = 0.0;
  start.acceleration = 0.0;

  goal.position = 2*M_PI;
  goal.velocity = 0.0;
  goal.acceleration = 0.0;

  path_generator_.calcCoefficient(start,
                                  goal,
                                  move_time,
                                  control_time);

  coefficient_ = path_generator_.getCoefficient();
}


Pose Circle::circle(float time_var)
{
  Pose pose;

  pose.position(0) = (initial_position_(0) - radius_) + (radius_ * cos(time_var));
  pose.position(1) = initial_position_(1) + (radius_ * sin(time_var));
  pose.position(2) = initial_position_(2);

  return pose;
}

Pose Circle::getPose(float tick)
{
  float get_time_var = 0.0;

  get_time_var = coefficient_(0) +
             coefficient_(1) * pow(tick, 1) +
             coefficient_(2) * pow(tick, 2) +
             coefficient_(3) * pow(tick, 3) +
             coefficient_(4) * pow(tick, 4) +
             coefficient_(5) * pow(tick, 5);

  return circle(get_time_var);;
}