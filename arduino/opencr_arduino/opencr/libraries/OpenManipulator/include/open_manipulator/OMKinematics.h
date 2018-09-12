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

/* Authors: Hye-Jong KIM, Darby Lim */

#ifndef OMKINEMATICS_H_
#define OMKINEMATICS_H_

#include <Eigen.h>  // Calls main Eigen matrix class library
#include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/QR>

#include <math.h>
#include <vector>
#include <map>

#include "OMAPI.h"
#include "OMDebug.h"

using namespace Eigen;

namespace OM_KINEMATICS
{
class Chain : public OPEN_MANIPULATOR::Kinematics
{
public:
  Chain(){};
  virtual ~Chain(){};

  virtual MatrixXf jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name);

  virtual void forward(OM_MANAGER::Manipulator *manipulator, Name component_name);
  virtual void forward(OM_MANAGER::Manipulator *manipulator);

  virtual std::vector<float> inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);

  std::vector<float> inverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<float> srInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<float> positionOnlyInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
};

class SCARA : public OPEN_MANIPULATOR::Kinematics
{
public:
  SCARA(){};
  virtual ~SCARA(){};

  virtual MatrixXf jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name);

  virtual void forward(OM_MANAGER::Manipulator *manipulator, Name component_name);
  virtual void forward(OM_MANAGER::Manipulator *manipulator);

  virtual std::vector<float> inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);

private:
  Chain chain_;
  std::vector<float> geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
};

class Link : public OPEN_MANIPULATOR::Kinematics
{
public:
  Link(){};
  virtual ~Link(){};

  virtual MatrixXf jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name);

  virtual void forward(OM_MANAGER::Manipulator *manipulator, Name component_name);
  virtual void forward(OM_MANAGER::Manipulator *manipulator);

  virtual std::vector<float> inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);

private:
  void solveKinematicsSinglePoint(OM_MANAGER::Manipulator *manipulator, Name component_name);
  std::vector<float> geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose); //for basic model);
};

class Planar : public OPEN_MANIPULATOR::Kinematics
{
public:
  Planar(){};
  virtual ~Planar(){};

  virtual MatrixXf jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name);

  virtual void forward(OM_MANAGER::Manipulator *manipulator, Name component_name);
  virtual void forward(OM_MANAGER::Manipulator *manipulator);

  virtual std::vector<float> inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);

private:
  std::vector<float> geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose); //for basic model);
};

} // namespace OM_KINEMATICS

#endif // OMKINEMATICS_HPP_