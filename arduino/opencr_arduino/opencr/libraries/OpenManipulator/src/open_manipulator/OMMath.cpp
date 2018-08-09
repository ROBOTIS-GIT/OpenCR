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

#include "../../include/open_manipulator/OMMath.h"

using namespace Eigen;

float MATH::sign(float number)
{
  if (number >= 0.0)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}

Vector3f MATH::makeVector3(float v1, float v2, float v3)
{
  Vector3f temp;
  temp << v1, v2, v3;
  return temp;
}

Matrix3f MATH::makeMatrix3(float m11, float m12, float m13,
                           float m21, float m22, float m23,
                           float m31, float m32, float m33)
{
  Matrix3f temp;
  temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return temp;
}

Vector3f MATH::matrixLogarithm(Matrix3f rotation_matrix)
{
  Matrix3f R = rotation_matrix;
  Vector3f l = Vector3f::Zero();
  Vector3f rotation_vector = Vector3f::Zero();

  float theta = 0.0;
  float diag = 0.0;
  bool diagonal_matrix = R.isDiagonal();

  l << R(2, 1) - R(1, 2),
      R(0, 2) - R(2, 0),
      R(1, 0) - R(0, 1);
  theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
  diag = R.determinant();

  if (R.isIdentity())
  {
    rotation_vector.setZero();
    return rotation_vector;
  }
  
  if (diagonal_matrix == true)
  {
    rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
    rotation_vector = rotation_vector * M_PI_2;
  }
  else
  {
    rotation_vector = theta * (l / l.norm());
  }
  return rotation_vector;
}

Matrix3f MATH::skewSymmetricMatrix(Vector3f v)
{
  Matrix3f skew_symmetric_matrix = Matrix3f::Zero();
  skew_symmetric_matrix << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return skew_symmetric_matrix;
}

Matrix3f MATH::rodriguesRotationMatrix(Vector3f axis, float angle)
{
  Matrix3f skew_symmetric_matrix = Matrix3f::Zero();
  Matrix3f rotation_matrix = Matrix3f::Zero();
  Matrix3f Identity_matrix = Matrix3f::Identity();

  skew_symmetric_matrix = skewSymmetricMatrix(axis);
  rotation_matrix = Identity_matrix +
                    skew_symmetric_matrix * sin(angle) +
                    skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));
  return rotation_matrix;
}

Matrix3f MATH::makeRotationMatrix(float roll, float pitch, float yaw)
{
#if 0 // Euler angle
  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();

  rotation_matrix << cos(yaw) * cos(pitch), (-1.0f) * sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll),
      sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll), (-1.0f) * cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll),
      (-1.0f) * sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll);

  return rotation_matrix;
#endif

  Vector3f rotation_vector;

  rotation_vector(0) = roll;
  rotation_vector(1) = pitch;
  rotation_vector(2) = yaw;

  return makeRotationMatrix(rotation_vector);
}

Matrix3f MATH::makeRotationMatrix(Vector3f rotation_vector)
{
  Matrix3f rotation_matrix;
  Vector3f axis;
  float angle;

  angle = rotation_vector.norm();
  axis(0) = rotation_vector(0) / angle;
  axis(1) = rotation_vector(1) / angle;
  axis(2) = rotation_vector(2) / angle;

  rotation_matrix = rodriguesRotationMatrix(axis, angle);
  return rotation_matrix;
}

Vector3f MATH::makeRotationVector(Matrix3f rotation_matrix)
{
  return matrixLogarithm(rotation_matrix);
}

Vector3f MATH::positionDifference(Vector3f desired_position, Vector3f present_position)
{
  Vector3f position_difference;
  position_difference = desired_position - present_position;

  return position_difference;
}

Vector3f MATH::orientationDifference(Matrix3f desired_orientation, Matrix3f present_orientation)
{
  Vector3f orientation_difference;
  orientation_difference = present_orientation * makeRotationVector(present_orientation.transpose() * desired_orientation);                   

  return orientation_difference;
}

VectorXf MATH::poseDifference(Vector3f desired_position, Vector3f present_position,
                              Matrix3f desired_orientation, Matrix3f present_orientation)
{
  Vector3f position_difference;
  Vector3f orientation_difference;
  VectorXf pose_difference(6);

  position_difference = positionDifference(desired_position, present_position);
  orientation_difference = orientationDifference(desired_orientation, present_orientation);
  pose_difference << position_difference(0), position_difference(1), position_difference(2),
      orientation_difference(0), orientation_difference(1), orientation_difference(2);

  return pose_difference;
}
