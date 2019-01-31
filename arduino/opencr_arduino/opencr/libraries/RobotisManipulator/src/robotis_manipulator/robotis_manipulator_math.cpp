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

#include "../../include/robotis_manipulator/robotis_manipulator_math.h"

/*****************************************************************************
** Make a Vector or Matrix
*****************************************************************************/
Eigen::Vector3d robotis_manipulator::math::vector3(double v1, double v2, double v3)
{
  Eigen::Vector3d temp;
  temp << v1, v2, v3;
  return temp;
}

Eigen::Matrix3d robotis_manipulator::math::matrix3(double m11, double m12, double m13,
                        double m21, double m22, double m23,
                        double m31, double m32, double m33)
{
  Eigen::Matrix3d temp;
  temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return temp;
}

Eigen::Matrix3d robotis_manipulator::math::inertiaMatrix(double ixx, double ixy, double ixz , double iyy , double iyz, double izz)
{
  Eigen::Matrix3d inertia;
  inertia <<
      ixx, ixy, ixz,
      ixy, iyy, iyz,
      ixz, iyz, izz;

  return inertia;
}


/*****************************************************************************
** Convert
*****************************************************************************/
// Translation Vector
Eigen::Vector3d robotis_manipulator::math::convertXYZToVector(double x, double y, double z)
{
  Eigen::Vector3d position;
  position << x, y, z;

  return position;
}

//Rotation
Eigen::Matrix3d robotis_manipulator::math::convertRollAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::Matrix3d robotis_manipulator::math::convertPitchAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::Matrix3d robotis_manipulator::math::convertYawAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      cos(angle), -sin(angle), 0.0,
      sin(angle), cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

Eigen::Vector3d robotis_manipulator::math::convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation)
{
  Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);
  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

  return rpy;
}

Eigen::Matrix3d robotis_manipulator::math::convertRPYToRotationMatrix(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d rotation = robotis_manipulator::math::convertYawAngleToRotationMatrix(yaw)*robotis_manipulator::math::convertPitchAngleToRotationMatrix(pitch)*robotis_manipulator::math::convertRollAngleToRotationMatrix(roll);

  return rotation;
}

Eigen::Quaterniond robotis_manipulator::math::convertRPYToQuaternion(double roll, double pitch, double yaw)
{
 Eigen::Quaterniond quaternion;
 quaternion = robotis_manipulator::math::convertRPYToRotationMatrix(roll,pitch,yaw);

 return quaternion;
}

Eigen::Quaterniond robotis_manipulator::math::convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation)
{
  Eigen::Quaterniond quaternion;
  quaternion = rotation;

  return quaternion;
}

Eigen::Vector3d robotis_manipulator::math::convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector3d rpy = robotis_manipulator::math::convertRotationMatrixToRPYVector(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Matrix3d robotis_manipulator::math::convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion)
{
  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  return rotation;
}

Eigen::Vector3d robotis_manipulator::math::convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix)
{
  return robotis_manipulator::math::matrixLogarithm(rotation_matrix);
}

// Transformation Matrix
Eigen::Matrix4d robotis_manipulator::math::convertXYZRPYToTransformationMatrix(double position_x, double position_y, double position_z , double roll , double pitch , double yaw)
{
  Eigen::Matrix4d transformation = robotis_manipulator::math::convertRPYToTransformationMatrix(roll, pitch, yaw);
  transformation.coeffRef(0,3) = position_x;
  transformation.coeffRef(1,3) = position_y;
  transformation.coeffRef(2,3) = position_z;

  return transformation;
}

Eigen::Matrix4d robotis_manipulator::math::convertXYZToTransformationMatrix(double position_x, double position_y, double position_z)
{
  Eigen::Matrix4d mat_translation;

  mat_translation <<
      1, 0, 0, position_x,
      0, 1, 0, position_y,
      0, 0, 1, position_z,
      0, 0, 0,          1;

  return mat_translation;
}

Eigen::Matrix4d robotis_manipulator::math::convertRPYToTransformationMatrix(double roll, double pitch, double yaw )
{
  double sr = sin(roll), cr = cos(roll);
  double sp = sin(pitch), cp = cos(pitch);
  double sy = sin(yaw), cy = cos(yaw);

  Eigen::Matrix4d mat_roll;
  Eigen::Matrix4d mat_pitch;
  Eigen::Matrix4d mat_yaw;

  mat_roll <<
      1, 0, 0, 0,
      0, cr, -sr, 0,
      0, sr, cr, 0,
      0, 0, 0, 1;

  mat_pitch <<
      cp, 0, sp, 0,
      0, 1, 0, 0,
      -sp, 0, cp, 0,
      0, 0, 0, 1;

  mat_yaw <<
      cy, -sy, 0, 0,
      sy, cy, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::Matrix4d mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

  return mat_rpy;
}

Eigen::Matrix4d robotis_manipulator::math::inverseTransformationMatrix(const Eigen::MatrixXd& transform)
{
  // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

  Eigen::Vector3d vec_boa;
  Eigen::Vector3d vec_x, vec_y, vec_z;
  Eigen::Matrix4d inv_t;

  vec_boa(0) = -transform(0,3);
  vec_boa(1) = -transform(1,3);
  vec_boa(2) = -transform(2,3);

  vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
  vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
  vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

  inv_t <<
      vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
      vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
      vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
      0, 0, 0, 1;

  return inv_t;
}

//Dynamic value
Eigen::Vector3d robotis_manipulator::math::convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega)
{
  Eigen::Matrix3d c_inverse;
  Eigen::Vector3d rpy_velocity;

  c_inverse << 1, sin(rpy_vector(0))*tan(rpy_vector(1)), cos(rpy_vector(0))*tan(rpy_vector(1)),
       0, cos(rpy_vector(0)),                    -sin(rpy_vector(0)),
       0, sin(rpy_vector(0))/cos(rpy_vector(1)), cos(rpy_vector(0))/cos(rpy_vector(1));

  rpy_velocity = c_inverse * omega;
  return rpy_velocity;
}

Eigen::Vector3d robotis_manipulator::math::convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity)
{
  Eigen::Matrix3d c;
  Eigen::Vector3d omega;

  c << 1, 0,                     -sin(rpy_vector(1)),
        0, cos(rpy_vector(0)),    sin(rpy_vector(0))*cos(rpy_vector(1)),
        0, -sin(rpy_vector(0)), cos(rpy_vector(0))*cos(rpy_vector(1));

  omega = c * rpy_velocity;
  return omega;
}

Eigen::Vector3d robotis_manipulator::math::convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot)
{
  Eigen::Vector3d c_dot;
  Eigen::Matrix3d c_inverse;
  Eigen::Vector3d rpy_acceleration;

  c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
           -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
           -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

  c_inverse << 1, sin(rpy_vector(0))*tan(rpy_vector(1)), cos(rpy_vector(0))*tan(rpy_vector(1)),
       0, cos(rpy_vector(0)),                    -sin(rpy_vector(0)),
       0, sin(rpy_vector(0))/cos(rpy_vector(1)), cos(rpy_vector(0))/cos(rpy_vector(1));

  rpy_acceleration = c_inverse * (omega_dot - c_dot);
  return rpy_acceleration;
}

Eigen::Vector3d robotis_manipulator::math::convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration)
{
  Eigen::Vector3d c_dot;
  Eigen::Matrix3d c;
  Eigen::Vector3d omega_dot;

  c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
           -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
           -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

  c << 1, 0,                     -sin(rpy_vector(1)),
        0, cos(rpy_vector(0)),    sin(rpy_vector(0))*cos(rpy_vector(1)),
        0, -sin(rpy_vector(0)), cos(rpy_vector(0))*cos(rpy_vector(1));

  omega_dot = c_dot + c * rpy_acceleration;
  return omega_dot;
}


/*****************************************************************************
** Math
*****************************************************************************/
double robotis_manipulator::math::sign(double value)
{
  if (value >= 0.0)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}

Eigen::Vector3d robotis_manipulator::math::matrixLogarithm(Eigen::Matrix3d rotation_matrix)
{
  Eigen::Matrix3d R = rotation_matrix;
  Eigen::Vector3d l = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();

  double theta = 0.0;
  // double diag = 0.0;
  bool diagonal_matrix = R.isDiagonal();

  l << R(2, 1) - R(1, 2),
      R(0, 2) - R(2, 0),
      R(1, 0) - R(0, 1);
  theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
  // diag = R.determinant();

  if (R.isIdentity())
  {
    rotation_vector.setZero(3);
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

Eigen::Matrix3d robotis_manipulator::math::skewSymmetricMatrix(Eigen::Vector3d v)
{
  Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
  skew_symmetric_matrix << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return skew_symmetric_matrix;
}

Eigen::Matrix3d robotis_manipulator::math::rodriguesRotationMatrix(Eigen::Vector3d axis, double angle)
{
  Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Identity_matrix = Eigen::Matrix3d::Identity();

  skew_symmetric_matrix = robotis_manipulator::math::skewSymmetricMatrix(axis);
  rotation_matrix = Identity_matrix +
                    skew_symmetric_matrix * sin(angle) +
                    skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));
  return rotation_matrix;
}

Eigen::Vector3d robotis_manipulator::math::positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position)
{
  Eigen::Vector3d position_difference;
  position_difference = desired_position - present_position;

  return position_difference;
}

Eigen::Vector3d robotis_manipulator::math::orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
  Eigen::Vector3d orientation_difference;
  orientation_difference = present_orientation * robotis_manipulator::math::matrixLogarithm(present_orientation.transpose() * desired_orientation);

  return orientation_difference;
}

Eigen::VectorXd robotis_manipulator::math::poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                              Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
  Eigen::Vector3d position_difference;
  Eigen::Vector3d orientation_difference;
  Eigen::VectorXd pose_difference(6);

  position_difference = robotis_manipulator::math::positionDifference(desired_position, present_position);
  orientation_difference = robotis_manipulator::math::orientationDifference(desired_orientation, present_orientation);
  pose_difference << position_difference(0), position_difference(1), position_difference(2),
      orientation_difference(0), orientation_difference(1), orientation_difference(2);

  return pose_difference;
}
