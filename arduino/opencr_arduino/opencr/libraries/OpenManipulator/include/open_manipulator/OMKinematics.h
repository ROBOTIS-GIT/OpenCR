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
#include <Eigen/Dense>

#include <math.h>
#include <vector>
#include <map>

#include "OMAPI.h"
#include "OMDebug.h"

using namespace Eigen;

namespace KINEMATICS
{
namespace CHAIN
{
MatrixXf jacobian(Manipulator *manipulator, int8_t tool_name);

void forward(Manipulator *manipulator);
VectorXf inverse(Manipulator *manipulator, int8_t tool_name);
} // namespace CHAIN

namespace LINK
{
void forward(Manipulator *manipulator);
VectorXf inverse(Manipulator *manipulator, int8_t tool_name);
} // namespace LINK

#if 0
class OMKinematicsMethod
{
public:
  OMKinematicsMethod(){};
  ~OMKinematicsMethod(){};

  void solveKinematicsSinglePoint(Manipulator *manipulator, Name component_name, bool *error = false);
  void forward(Manipulator *manipulator, bool *error = false);
  MatrixXf jacobian(Manipulator *manipulator, int8_t tool_component_name, bool *error = false);
};

class OMChainKinematics
{
private:
public:
  OMChainKinematics(){};
  ~OMChainKinematics(){};
};

class OMLinkKinematics
{
private:
public:
  OMLinkKinematics(){};
  ~OMLinkKinematics(){};

  void forward(Manipulator *manipulator, bool *error = false);
  VectorXf geometricInverse(Manipulator *manipulator, Name tool_number, Pose target_pose, bool *error = false); //for basic model
};
#endif
} // namespace KINEMATICS

#endif // OMKINEMATICS_HPP_