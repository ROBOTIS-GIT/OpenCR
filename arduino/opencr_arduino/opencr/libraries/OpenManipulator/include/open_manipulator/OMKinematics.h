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

namespace OM_KINEMATICS
{
namespace CHAIN
{
MatrixXf jacobian(Manipulator *manipulator, Name tool_name);

void forward(Manipulator *manipulator, Name component_name);

std::vector<float> inverse(Manipulator *manipulator, Name tool_name, Pose target_pose);
std::vector<float> sr_inverse(Manipulator *manipulator, Name tool_name, Pose target_pose);
std::vector<float> position_only_inverse(Manipulator *manipulator, Name tool_name, Pose target_pose);
} // namespace CHAIN

namespace LINK
{
void solveKinematicsSinglePoint(Manipulator *manipulator, Name component_name);
void forward(Manipulator *manipulator);
std::vector<float> geometricInverse(Manipulator *manipulator, Name tool_name, Pose target_pose); //for basic model);
} // namespace LINK
} // namespace KINEMATICS

#endif // OMKINEMATICS_HPP_