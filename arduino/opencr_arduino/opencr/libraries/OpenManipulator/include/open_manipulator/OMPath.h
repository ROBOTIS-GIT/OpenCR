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

#ifndef OMPATH_H_
#define OMPATH_H_

#include <Eigen.h>  // Calls main Eigen matrix class library
#include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/QR>

#include <math.h>
#include <vector>

#include "OMDebug.h"

using namespace Eigen;

typedef struct
{
  float position;
  float velocity;
  float acceleration;
} Trajectory;

namespace OM_PATH
{
class MinimumJerk
{
private:
  VectorXf coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  void calcCoefficient(Trajectory start,
                       Trajectory goal,
                       float move_time,
                       float control_time);

  VectorXf getCoefficient();
};

class JointTrajectory
{
private:
  MinimumJerk path_generator_;

  uint8_t joint_num_;
  MatrixXf coefficient_;
  std::vector<float> position_;
  std::vector<float> velocity_;
  std::vector<float> acceleration_;

public:
  JointTrajectory(uint8_t joint_num);
  virtual ~JointTrajectory();

  void init(std::vector<Trajectory> start,
            std::vector<Trajectory> goal,
            float move_time,
            float control_time);

  std::vector<float> getPosition(float tick);
  std::vector<float> getVelocity(float tick);
  std::vector<float> getAcceleration(float tick);

  MatrixXf getCoefficient();
};

class DrawCircle
{
private:
  MinimumJerk path_generator_;

  uint8_t joint_num_;
  MatrixXf coefficient_;
  std::vector<float> position_;
  std::vector<float> velocity_;
  std::vector<float> acceleration_;

public:
  DrawCircle(uint8_t joint_num);
  virtual ~DrawCircle();

  void init(std::vector<Trajectory> start,
            std::vector<Trajectory> goal,
            float move_time,
            float control_time);

  std::vector<float> getPosition(float tick);
  std::vector<float> getVelocity(float tick);
  std::vector<float> getAcceleration(float tick);

  MatrixXf getCoefficient();
};
} // namespace PATH
#endif // OMPATH_H_
