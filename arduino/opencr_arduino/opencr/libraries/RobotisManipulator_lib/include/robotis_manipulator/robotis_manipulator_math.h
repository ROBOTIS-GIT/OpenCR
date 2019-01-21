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

#define DEG2RAD 0.01745329252f //(M_PI / 180.0)
#define RAD2DEG 57.2957795131f //(180.0 / M_PI)

#define ZERO_VECTOR Eigen::Vector3d::Zero()
#define IDENTITY_MATRIX Eigen::Matrix3d::Identity(3, 3)

namespace RM_MATH
{
double sign(double number);

Eigen::Vector3d makeVector3(double v1, double v2, double v3);
Eigen::Matrix3d makeMatrix3(double m11, double m12, double m13,
                            double m21, double m22, double m23,
                            double m31, double m32, double m33);

Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z);
Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z , double roll, double pitch, double yaw);
Eigen::Matrix4d getInverseTransformation(const Eigen::MatrixXd& transform);
Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);
Eigen::Matrix3d getRotationX(double angle);
Eigen::Matrix3d getRotationY(double angle);
Eigen::Matrix3d getRotationZ(double angle);
Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw);
Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z);

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation);
Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation);
Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion);
Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion);
Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d& rotation_matrix);

Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix);
Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v);
Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle);

Eigen::Vector3d getRPYVelocityFromOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega);
Eigen::Vector3d getOmegaFromRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity);
Eigen::Vector3d getRPYAccelerationFromOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot);
Eigen::Vector3d getOmegaDotFromRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration);


Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position);
Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                        Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);

template <typename T> T map(T x, T in_min, T in_max, T out_min, T out_max);

} // namespace RM_MATH

#endif // ROBOTIS_MANIPULATOR_MATH_H_
