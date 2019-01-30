/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef ROBOTIS_MANIPULATOR_MATH_H_
#define ROBOTIS_MANIPULATOR_MATH_H_

#include <unistd.h>

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <Eigen/Geometry>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
#endif

#include <math.h>

#define DEG2RAD 0.01745329252 //(M_PI / 180.0)
#define RAD2DEG 57.2957795131 //(180.0 / M_PI)

namespace robotis_manipulator_math
{

/*****************************************************************************
** Make a Vector or Matrix
*****************************************************************************/
Eigen::Vector3d vector3(double v1, double v2, double v3);
Eigen::Matrix3d matrix3(double m11, double m12, double m13,
                        double m21, double m22, double m23,
                        double m31, double m32, double m33);
Eigen::Matrix3d inertiaMatrix(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);


/*****************************************************************************
** Convert
*****************************************************************************/
// Translation Vector
Eigen::Vector3d convertXYZ2Vector(double x, double y, double z);

// Rotation 
Eigen::Matrix3d convertRollAngle2RotationMatrix(double angle);
Eigen::Matrix3d convertPitchAngle2RotationMatrix(double angle);
Eigen::Matrix3d convertYawAngle2RotationMatrix(double angle);
Eigen::Vector3d convertRotationMatrix2RPYVector(const Eigen::Matrix3d& rotation_matrix);
Eigen::Matrix3d convertRPY2RotationMatrix(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPY2Quaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationMatrix2Quaternion(const Eigen::Matrix3d& rotation_matrix);
Eigen::Vector3d convertQuaternion2RPYVector(const Eigen::Quaterniond& quaternion);
Eigen::Matrix3d convertQuaternion2RotationMatrix(const Eigen::Quaterniond& quaternion);
Eigen::Vector3d convertRotationMatrix2Omega(const Eigen::Matrix3d& rotation_matrix);

// Transformation Matrix
Eigen::Matrix4d convertXYZRPY2TransformationMatrix(double x, double y, double z , double roll, double pitch, double yaw);
Eigen::Matrix4d convertXYZ2TransformationMatrix(double x, double y, double z);
Eigen::Matrix4d convertRPY2TransformationMatrix(double roll, double pitch, double yaw);

// Dynamic Value
Eigen::Vector3d convertOmega2RPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega);
Eigen::Vector3d convertRPYVelocity2Omega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity);
Eigen::Vector3d convertOmegaDot2RPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot);
Eigen::Vector3d convertRPYAcceleration2OmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration);


/*****************************************************************************
** Math
*****************************************************************************/
double sign(double value);

Eigen::Matrix4d inverseTransformationMatrix(const Eigen::MatrixXd& transformation_matrix);
Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix);
Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v);
Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle);

Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position);
Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                        Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
} // namespace robotis_manipulator_math

#endif // ROBOTIS_MANIPULATOR_MATH_H_
