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

/* Authors: Hye-Jong KIM */

#ifndef OMKINEMATICS_H_
#define OMKINEMATICS_H_

#include "../../include/open_manipulator/OMAPI.h"
#include "../../include/open_manipulator/OMDebug.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>
#include <math.h>
#include <vector>

using namespace OPEN_MANIPULATOR;

class OMKinematicsMethod
{
 public:
  OMKinematicsMethod(){};
  ~OMKinematicsMethod(){};

  void solveKinematicsSinglePoint(Manipulator* manipulator, Name component_name, bool* error = false);
  void forward(Manipulator* manipulator, bool* error = false);
  Eigen::MatrixXf jacobian(Manipulator* manipulator, int8_t tool_component_name, bool *error = false);
};

class OMChainKinematics
{
 private:

 public:
  OMChainKinematics(){};
  ~OMChainKinematics(){};
};

class OMScaraKinematics
{
  private:

  public:
    OMScaraKinematics(){};
    ~OMScaraKinematics(){};
};

class OMLinkKinematics
{
 private:

 public:
  OMLinkKinematics(){};
  ~OMLinkKinematics(){};

  void forward(Manipulator* manipulator, bool *error = false);
  Eigen::VectorXf geometricInverse(Manipulator* manipulator, Name tool_number, Pose target_pose, float gain); //for basic model
};

class OMDeltaKinematics
{
  private:

  public:
    OMDeltaKinematics(){};
    ~OMDeltaKinematics(){};

};

class MYKinematics
{
 private:

 public:
   MYKinematics(){};
   ~MYKinematics(){};
};

#endif // OMKINEMATICS_HPP_
