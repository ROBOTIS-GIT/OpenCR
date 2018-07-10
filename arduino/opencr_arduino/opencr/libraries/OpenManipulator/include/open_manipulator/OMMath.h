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

#ifndef OMMATH_H_
#define OMMATH_H_

#include "../../include/open_manipulator/OMDebug.h"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class OMMath
{
 public:
  OMMath();
  ~OMMath();
  float sign(float number);
  Eigen::Vector3f makeEigenVector3(float v1, float v2, float v3);
  Eigen::Matrix3f makeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);
  Eigen::Matrix3f makeRotationMatrix(float roll, float pitch, float yaw);
  
  Eigen::Matrix3f skewSymmetricMatrix(Eigen::Vector3f v);
  Eigen::Matrix3f rodriguesRotationMatrix(Eigen::Vector3f axis, float angle);
  
  
};

#endif // OMMATH_H_
