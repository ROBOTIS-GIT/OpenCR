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

#ifndef OMMATH_HPP_
#define OMMATH_HPP_

#include "../../include/open_manipulator/OMDebug.hpp"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class OMMath
{
 public:
  OMMath(){}
  ~OMMath(){}

  float sign(float number)
  {
    if (num >= 0.0)
    {
      return 1.0;
    }
    else
    {
      return -1.0;
    }
  }

  Eigen::Vector3f makeEigenVector3(float v1, float v2, float v3)
  {
    Eigen::Vector3f temp;
    temp << v1, v2, v3;
    return temp;
  }

  Eigen::Matrix3f makeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
  {
    Eigen::Matrix3f temp;
    temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    return temp;
  }
  
  Eigen::Matrix3f makeRotationMatrix(float roll, float pitch, float yaw)
  {
    Eigen::Matrix3f rotation_matrix;
    Eigen::Matrix3f roll_matrix;
    Eigen::Matrix3f pitch_matrix;
    Eigen::Matrix3f yaw_matrix;

    roll_matrix  << 1.000,  0.000,     0.000,
                    0.000,  cos(roll), sin(roll),
                    0.000, -sin(roll), cos(roll);

    pitch_matrix << cos(pitch),  0.000, sin(pitch),
                    0.000,       1.000, 0.000,
                    -sin(pitch), 0.000, cos(pitch);

    yaw_matrix   << cos(yaw), -sin(yaw), 0.000,
                    sin(yaw), cos(yaw),  0.000,
                    0.000,    0.000,     1.000;
    
    rotation_matrix = roll_matrix * pitch_matrix * yaw_matrix;

    return rotation_matrix;
  }

  Eigen::Matrix3f makeRotationMatrix(Eigen::Vector3f rotation_vector)
  {
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = makeRotationMatrix(rotation_vector(0), rotation_vector(1), rotation_vector(2));
    return rotation_matrix;
  }
  
  Eigen::Vector3f makeRotationVector(Eigen::Matrix3f rotation_matrix)
  {
    Eigen::Matrix3f R = rotation_matrix;
    Eigen::Vector3f l = Eigen::Vector3f::Zero();
    Eigen::Vector3f rotation_vector = Eigen::Vector3f::Zero();

    float theta = 0.0;
    float diag  = 0.0;
    bool diagonal_matrix = true;

    l << R(2,1) - R(1,2),
         R(0,2) - R(2,0),
         R(1,0) - R(0,1);
    theta = atan2(l.norm(), R(0,0) + R(1,1) + R(2,2) - 1);
    diag  = R(0,0) + R(1,1) + R(2,2);

    for (int8_t i = 0; i < 3; i++)
    {
      for (int8_t j = 0; j < 3; j++)
      {
        if (i != j)
        {
          if (R(i, j) != 0)
          {
            diagonal_matrix = false;
          }
        }
      }
    }
    if (R == Eigen::Matrix3f::Identity())
    {
      rotation_vector = Eigen::Vector3f::Zero();
    }
    else if (diagonal_matrix == true)
    {
      rotation_vector << R(0,0) + 1, R(1,1) + 1, R(2,2) + 1;
      rotation_vector = rotation_vector * M_PI_2;
    }
    else
    {
      rotation_vector = theta * (l / l.norm());
    }
    return rotation_vector;
  }

  Eigen::Matrix3f skewSymmetricMatrix(Eigen::Vector3f v)
  {
    Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();

    skew_symmetric_matrix <<    0,  -v(2),  v(1),
                            v(2),      0, -v(0),
                            -v(1),   v(0),     0;

    return skew_symmetric_matrix;
  }

  Eigen::Matrix3f rodriguesRotationMatrix(Eigen::Vector3f axis, float angle)
  {
    Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f Identity_matrix = Eigen::Matrix3f::Identity();

    skew_symmetric_matrix = skewSymmetricMatrix(axis);
    rotation_matrix = Identity_matrix +
                      skew_symmetric_matrix * sin(angle) +
                      skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));

    return rotation_matrix;
  }
  
  Eigen::Vector3f differentialPosition(Eigen::Vector3f desired_position, Eigen::Vector3f present_position)
  {
    Eigen::Vector3f differential_position;
    differential_position = desired_position - present_position;

    return differential_position;
  }

  Eigen::Vector3f differentialOrientation(Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation)
  {
    Eigen::Vector3f differential_orientation;
    differential_orientation = present_orientation * makeRotationVector(present_orientation.transpose() * desired_orientation);

    return differential_orientation;
  }


  Eigen::VectorXf differentialPose(Eigen::Vector3f desired_position, Eigen::Vector3f present_position, Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation)
  {
    Eigen::Vector3f differential_position;
    Eigen::Vector3f differential_orientation;
    Eigen::VectorXf  differential_pose(6);

    differential_position = differentialPosition(desired_position, present_position);
    differential_orientation = differentialOrientation(desired_orientation, present_orientation)
    differential_pose << differential_position(0),
                         differential_position(1),
                         differential_position(2),
                         differential_orientation(0),
                         differential_orientation(1),
                         differential_orientation(2);
    return differential_pose;
  }
};

#endif // OMMATH_HPP_
