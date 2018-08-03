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

#ifndef OMMATH_H_
#define OMMATH_H_

#include <unistd.h>
#include <Eigen.h>  // Calls main Eigen matrix class library
#include <Eigen/Geometry>
#include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.

#include <math.h>

using namespace Eigen;

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define ZERO_VECTOR Vector3f::Zero()
#define IDENTITY_MATRIX Matrix3f::Identity(3, 3)

namespace MATH
{
float sign(float number);

Vector3f makeVector3(float v1, float v2, float v3);
Matrix3f makeMatrix3(float m11, float m12, float m13,
                     float m21, float m22, float m23,
                     float m31, float m32, float m33);

Vector3f matrixLogarithm(Matrix3f rotation_matrix);
Matrix3f skewSymmetricMatrix(Vector3f v);
Matrix3f rodriguesRotationMatrix(Vector3f axis, float angle);

Matrix3f makeRotationMatrix(float roll, float pitch, float yaw);
Matrix3f makeRotationMatrix(Vector3f rotation_vector);
Vector3f makeRotationVector(Matrix3f rotation_matrix);

Vector3f positionDifference(Vector3f desired_position, Vector3f present_position);
Vector3f orientationDifference(Matrix3f desired_orientation, Matrix3f present_orientation);
VectorXf poseDifference(Vector3f desired_position, Vector3f present_position,
                        Matrix3f desired_orientation, Matrix3f present_orientation);
} // namespace MATH

#endif // OMMATH_HPP_
