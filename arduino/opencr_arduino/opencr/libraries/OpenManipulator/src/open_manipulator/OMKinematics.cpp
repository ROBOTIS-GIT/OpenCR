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

#include "../../include/open_manipulator/OMKinematics.h"

using namespace Eigen;
using namespace OPEN_MANIPULATOR;
using namespace OM_KINEMATICS;

MatrixXf Chain::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(manipulator));

  Vector3f joint_axis = ZERO_VECTOR;

  Vector3f position_changed = ZERO_VECTOR;
  Vector3f orientation_changed = ZERO_VECTOR;
  VectorXf pose_changed = VectorXf::Zero(6);

  int8_t index = 0;
  Name my_name = getIteratorBegin(manipulator)->first;

  for (int8_t size = 0; size < getDOF(manipulator); size++)
  {
    Name parent_name = getComponentParentName(manipulator, my_name);
    if (parent_name == getWorldName(manipulator))
    {
      joint_axis = getWorldOrientation(manipulator) * getComponentJointAxis(manipulator, my_name);
    }
    else
    {
      joint_axis = getComponentOrientationToWorld(manipulator, parent_name) * getComponentJointAxis(manipulator, my_name);
    }

    position_changed = OM_MATH::skewSymmetricMatrix(joint_axis) *
                       (getComponentPositionToWorld(manipulator, tool_name) - getComponentPositionToWorld(manipulator, my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = getComponentChildName(manipulator, my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void Chain::forward(OM_MANAGER::Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void Chain::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = getComponentParentName(manipulator, my_name);
  int8_t number_of_child = getComponentChildName(manipulator, my_name).size();

  Vector3f parent_position_to_world, my_position_to_world;
  Matrix3f parent_orientation_to_world, my_orientation_to_world;

  if (parent_name == getWorldName(manipulator))
  {
    parent_position_to_world = getWorldPosition(manipulator);
    parent_orientation_to_world = getWorldOrientation(manipulator);
  }
  else
  {
    parent_position_to_world = getComponentPositionToWorld(manipulator, parent_name);
    parent_orientation_to_world = getComponentOrientationToWorld(manipulator, parent_name);
  }

  my_position_to_world = parent_orientation_to_world * getComponentRelativePositionToParent(manipulator, my_name) + parent_position_to_world;
  my_orientation_to_world = parent_orientation_to_world * OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, my_name), getComponentJointAngle(manipulator, my_name));

  setComponentPositionToWorld(manipulator, my_name, my_position_to_world);
  setComponentOrientationToWorld(manipulator, my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = getComponentChildName(manipulator, my_name).at(index);
    forward(manipulator, child_name);
  }
}

std::vector<float> Chain::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  // return positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
  return srInverseKinematics(manipulator, tool_name, target_pose);
}

std::vector<float> Chain::inverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  const float lambda = 0.7;
  const int8_t iteration = 10;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));

  VectorXf pose_changed = VectorXf::Zero(6);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));

  for (int8_t count = 0; count < iteration; count++)
  {
    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);

    jacobian = this->jacobian(&_manipulator, tool_name);

    pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
    if (pose_changed.norm() < 1E-6)
      return getAllActiveJointAngle(&_manipulator);

    ColPivHouseholderQR<MatrixXf> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<float> Chain::srInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  float lambda = 0.0;
  const float param = 0.002;
  const int8_t iteration = 50;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf pose_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  float wn_pos = 1 / 0.3;
  float wn_ang = 1 / (2 * M_PI);
  float Ek = 0.0;
  float Ek2 = 0.0;

  MatrixXf We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                         target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));
  Ek = pose_changed.transpose() * We * pose_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = Ek + param;

    updated_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = jacobian.transpose() * We * pose_changed;

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    pose_changed = OM_MATH::poseDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name),
                                           target_pose.orientation, getComponentOrientationToWorld(&_manipulator, tool_name));

    Ek2 = pose_changed.transpose() * We * pose_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<float> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}

std::vector<float> Chain::positionOnlyInverseKinematics(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  float lambda = 0.0;
  const float param = 0.002;
  const int8_t iteration = 10;

  OM_MANAGER::Manipulator _manipulator = *manipulator;

  MatrixXf jacobian = MatrixXf::Identity(6, getDOF(&_manipulator));
  MatrixXf position_jacobian = MatrixXf::Identity(3, getDOF(&_manipulator));
  MatrixXf updated_jacobian = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));
  VectorXf position_changed = VectorXf::Zero(3);
  VectorXf angle_changed = VectorXf::Zero(getDOF(&_manipulator));
  VectorXf gerr(getDOF(&_manipulator));

  float wn_pos = 1 / 0.3;
  float wn_ang = 1 / (2 * M_PI);
  float Ek = 0.0;
  float Ek2 = 0.0;

  MatrixXf We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXf Wn = MatrixXf::Identity(getDOF(&_manipulator), getDOF(&_manipulator));

  forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
  position_changed = OM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));
  Ek = position_changed.transpose() * We * position_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = Ek + param;

    updated_jacobian = (position_jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = position_jacobian.transpose() * We * position_changed;

    ColPivHouseholderQR<MatrixXf> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<float> set_angle_changed;
    for (int8_t index = 0; index < getDOF(&_manipulator); index++)
      set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) + angle_changed(index));

    setAllActiveJointAngle(&_manipulator, set_angle_changed);

    forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    position_changed = OM_MATH::positionDifference(target_pose.position, getComponentPositionToWorld(&_manipulator, tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return getAllActiveJointAngle(&_manipulator);
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<float> set_angle_changed;
      for (int8_t index = 0; index < getDOF(&_manipulator); index++)
        set_angle_changed.push_back(getAllActiveJointAngle(&_manipulator).at(index) - angle_changed(index));

      setAllActiveJointAngle(&_manipulator, set_angle_changed);

      forward(&_manipulator, getIteratorBegin(&_manipulator)->first);
    }
  }

  return getAllActiveJointAngle(&_manipulator);
}

MatrixXf SCARA::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
  chain_.jacobian(manipulator, tool_name);
}

void SCARA::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  chain_.forward(manipulator, component_name);
}

void SCARA::forward(OM_MANAGER::Manipulator *manipulator)
{  
  chain_.forward(manipulator);
}

std::vector<float> SCARA::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return chain_.positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
}




// std::vector<float> SCARA::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
// {
//   std::vector<float> target_angle_vector;

//   double target_angle[3];
//   double link[3];
//   double temp_x[3];
//   double temp_y[3];
//   double target_pose_length[3];

//   // Link Lengths
//   link[0] = 0.067f;
//   link[1] = 0.067f;
//   link[2] = 0.107f;

//   // Pose for each set of two joints
//   temp_x[0] = target_pose.position(0);
//   temp_y[0] = target_pose.position(1);

//   temp_x[1] = cos(PI*2/3)*temp_x[0] + sin(PI*2/3)*temp_y[0];
//   temp_y[1] = -sin(PI*2/3)*temp_x[0] + cos(PI*2/3)*temp_y[0];

//   temp_x[2] = cos(PI*2/3)*temp_x[1] + sin(PI*2/3)*temp_y[1];
//   temp_y[2] = -sin(PI*2/3)*temp_x[1] + cos(PI*2/3)*temp_y[1];

//   // Length of Position Difference and Target Angle
//   target_pose_length[0] = sqrt(temp_x[0]*temp_x[0] + (temp_y[0]+0.1339)*(temp_y[0]+0.1339));
//   target_angle[0] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[0]*target_pose_length[0]) 
//                           / (2*link[0]*target_pose_length[0]));

//   target_pose_length[1] = sqrt(temp_x[1]*temp_x[1] + (temp_y[1]+0.1339)*(temp_y[1]+0.1339));
//   target_angle[1] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[1]*target_pose_length[1]) 
//                           / (2*link[0]*target_pose_length[1]));

//   target_pose_length[2] = sqrt(temp_x[2]*temp_x[2] + (temp_y[2]+0.1339)*(temp_y[2]+0.1339));
//   target_angle[2] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[2]*target_pose_length[2]) 
//                           / (2*link[0]*target_pose_length[2]));
//   // Set Joint Angle 
//   target_angle_vector.push_back(target_angle[0] - PI/4);
//   target_angle_vector.push_back(target_angle[1] - PI/4);
//   target_angle_vector.push_back(target_angle[2] - PI/4);

//   // For Testing..
//   // Serial.println(target_angle[0] - PI/4, 5);
//   // Serial.println(target_angle[1] - PI/4, 5);
//   // Serial.println(target_angle[2] - PI/4, 5);

//   // target_angle_vector.push_back(0);
//   // target_angle_vector.push_back(0);
//   // target_angle_vector.push_back(0);

//   return target_angle_vector;
// }





















MatrixXf Link::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Link::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

void Link::forward(OM_MANAGER::Manipulator *manipulator)
{
  Pose pose_to_wolrd;
  Pose link_relative_pose;
  Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  //Base Pose Set (from world)
  pose_to_wolrd = getWorldPose(manipulator);
  link_relative_pose = getComponentRelativePoseToParent(manipulator, getWorldChildName(manipulator));
  rodrigues_rotation_matrix = OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, getWorldChildName(manipulator)), getComponentJointAngle(manipulator, getWorldChildName(manipulator)));

  result_pose.position = pose_to_wolrd.position + pose_to_wolrd.orientation * link_relative_pose.position;
  result_pose.orientation = pose_to_wolrd.orientation * link_relative_pose.orientation * rodrigues_rotation_matrix;
  setComponentPoseToWorld(manipulator, getWorldChildName(manipulator), result_pose);

  //Next Component Pose Set
  for (int i = 0; i < getComponentChildName(manipulator, getWorldChildName(manipulator)).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, getWorldChildName(manipulator)).at(i));
  }
}

std::vector<float> Link::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

void Link::solveKinematicsSinglePoint(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
  Pose parent_pose;
  Pose link_relative_pose;
  Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  parent_pose = getComponentPoseToWorld(manipulator, getComponentParentName(manipulator, component_name));
  link_relative_pose = getComponentRelativePoseToParent(manipulator, component_name);
  rodrigues_rotation_matrix = OM_MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, component_name), getComponentJointAngle(manipulator, component_name));

  result_pose.position = parent_pose.position + parent_pose.orientation * link_relative_pose.position;
  result_pose.orientation = parent_pose.orientation * link_relative_pose.orientation * rodrigues_rotation_matrix;

  setComponentPoseToWorld(manipulator, component_name, result_pose);
  for (int i = 0; i < getComponentChildName(manipulator, component_name).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, component_name).at(i));
  }
}

std::vector<float> Link::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose) //for basic model
{
  std::vector<float> target_angle_vector;
  Vector3f control_position; //joint6-joint1
  Vector3f tool_relative_position = getComponentRelativePositionToParent(manipulator, tool_name);
  Vector3f base_position = getComponentPositionToWorld(manipulator, getWorldChildName(manipulator));
  Vector3f temp_vector;

  float target_angle[3];
  float link[3];
  float temp_x;
  float temp_y;

  temp_x = target_pose.position(0) - base_position(0);
  temp_y = target_pose.position(1) - base_position(1);
  target_angle[0] = atan2(temp_y, temp_x);

  control_position(0) = target_pose.position(0) - tool_relative_position(0) * cos(target_angle[0]);
  control_position(1) = target_pose.position(1) - tool_relative_position(0) * sin(target_angle[0]);
  control_position(2) = target_pose.position(2) - tool_relative_position(2);

  // temp_vector = omlink.link_[0].getRelativeJointPosition(1,0);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, tool_name))));
  link[0] = temp_vector(2);
  // temp_vector = omlink.link_[1].getRelativeJointPosition(5,1);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, getComponentParentName(manipulator, tool_name)));
  link[1] = temp_vector(0);
  // temp_vector = omlink.link_[4].getRelativeJointPosition(6,5);
  temp_vector = getComponentRelativePositionToParent(manipulator, getComponentParentName(manipulator, tool_name));
  link[2] = temp_vector(0);

  temp_y = control_position(2) - base_position(2) - link[0];
  temp_x = (control_position(0) - base_position(0)) / cos(target_angle[0]);

  target_angle[1] = acos(((temp_x * temp_x + temp_y * temp_y + link[1] * link[1] - link[2] * link[2])) / (2 * link[1] * sqrt(temp_x * temp_x + temp_y * temp_y))) + atan2(temp_y, temp_x);
  target_angle[2] = acos((link[1] * link[1] + link[2] * link[2] - (temp_x * temp_x + temp_y * temp_y)) / (2 * link[1] * link[2])) + target_angle[1];

  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(-target_angle[1]);
  target_angle_vector.push_back(-target_angle[2]);

  return target_angle_vector;
}














//--------------------------Planar below---------------------------//


MatrixXf Planar::jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name)
{
}

void Planar::forward(OM_MANAGER::Manipulator *manipulator)
{
}

void Planar::forward(OM_MANAGER::Manipulator *manipulator, Name component_name)
{
}

std::vector<float> Planar::inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  return geometricInverse(manipulator, tool_name, target_pose);
}

std::vector<float> Planar::geometricInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  std::vector<float> target_angle_vector;

  double target_angle[3];
  double link[2];
  double temp_x[3];       // why not use double...?
  double temp_y[3];
  double target_pose_length[3];

  // Link Lengths
  link[0] = 0.120f;
  link[1] = 0.098f;

  // Pose for each set of two joints
  temp_x[0] = target_pose.position(0);
  temp_y[0] = target_pose.position(1);

  temp_x[1] = cos(PI*2/3)*temp_x[0] + sin(PI*2/3)*temp_y[0];
  temp_y[1] = -sin(PI*2/3)*temp_x[0] + cos(PI*2/3)*temp_y[0];

  temp_x[2] = cos(PI*2/3)*temp_x[1] + sin(PI*2/3)*temp_y[1];
  temp_y[2] = -sin(PI*2/3)*temp_x[1] + cos(PI*2/3)*temp_y[1];

  // Length of Position Difference and Target Angle
  target_pose_length[0] = sqrt(temp_x[0]*temp_x[0] + (temp_y[0]+0.1339)*(temp_y[0]+0.1339));
  target_angle[0] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[0]*target_pose_length[0]) 
                          / (2*link[0]*target_pose_length[0]));

  target_pose_length[1] = sqrt(temp_x[1]*temp_x[1] + (temp_y[1]+0.1339)*(temp_y[1]+0.1339));
  target_angle[1] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[1]*target_pose_length[1]) 
                          / (2*link[0]*target_pose_length[1]));

  target_pose_length[2] = sqrt(temp_x[2]*temp_x[2] + (temp_y[2]+0.1339)*(temp_y[2]+0.1339));
  target_angle[2] = asin((-link[1]*link[1] + link[0]*link[0] + target_pose_length[2]*target_pose_length[2]) 
                          / (2*link[0]*target_pose_length[2]));
  // Set Joint Angle 
  target_angle_vector.push_back(target_angle[0] - PI/4);
  target_angle_vector.push_back(target_angle[1] - PI/4);
  target_angle_vector.push_back(target_angle[2] - PI/4);

  // For Testing..
  // Serial.println(target_angle[0] - PI/4, 5);
  // Serial.println(target_angle[1] - PI/4, 5);
  // Serial.println(target_angle[2] - PI/4, 5);

  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);
  // target_angle_vector.push_back(0);

  return target_angle_vector;
}