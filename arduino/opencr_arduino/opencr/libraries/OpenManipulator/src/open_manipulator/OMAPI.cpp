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

/* Authors: Darby Limm, Hye-Jong KIM */

#include "../../include/open_manipulator/OMAPI.h"

using namespace OPEN_MANIPULATOR;

#if 0

////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

//math
float MATH::sign(float number)
{
  return OMMath::sign(number);
}

Eigen::Vector3f MATH::makeEigenVector3(float v1, float v2, float v3)
{
  return OMMath::makeEigenVector3(v1, v2, v3);
}

Eigen::Matrix3f MATH::makeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
{
  return OMMath::makeEigenMatrix3(m11, m12, m13, m21, m22, m23, m31, m32, m33);
}
  
Eigen::Vector3f MATH::matrixLogarithm(Eigen::Matrix3f rotation_matrix)
{
  return OMMath::matrixLogarithm(rotation_matrix);
}

Eigen::Matrix3f MATH::skewSymmetricMatrix(Eigen::Vector3f v)
{
  return OMMath::skewSymmetricMatrix(v);
}

Eigen::Matrix3f MATH::rodriguesRotationMatrix(Eigen::Vector3f axis, float angle)
{
  return OMMath::rodriguesRotationMatrix(axis, angle);
}

Eigen::Matrix3f MATH::makeRotationMatrix(float rool, float pitch, float yaw)
{
  return OMMath::makeRotationMatrix(rool, pitch, yaw);
}

Eigen::Matrix3f MATH::makeRotationMatrix(Eigen::Vector3f rotation_vector)
{
  return OMMath::makeRotationMatrix(rotation_vector);
}

Eigen::Vector3f MATH::makeRotationVector(Eigen::Matrix3f rotation_matrix)
{
  return OMMath::makeRotationVector(rotation_matrix);
}

Eigen::Vector3f MATH::differentialPosition(Eigen::Vector3f desired_position, Eigen::Vector3f present_position)
{
  return OMMath::differentialPosition(desired_position, present_position);
}

Eigen::Vector3f MATH::differentialOrientation(Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation)
{
  return OMMath::differentialOrientation(desired_orientation, present_orientation);
}

Eigen::VectorXf MATH::differentialPose(Eigen::Vector3f desired_position, Eigen::Vector3f present_position, Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation)
{
  return OMMath::differentialPose(desired_position, present_position, desired_orientation, present_orientation);
}
#endif

//manager
void MANAGER::addWorld(Manipulator *manipulator,
                       Name world_name,
                       Name child_name,
                       Vector3f world_position,
                       Matrix3f world_orientation)
{
  manipulator->addWorld(world_name, child_name, world_position, world_orientation);
}

void MANAGER::addComponent(Manipulator *manipulator,
                           Name my_name,
                           Name parent_name,
                           Name child_name,
                           Vector3f relative_position,
                           Matrix3f relative_orientation,
                           Vector3f axis_of_rotation,
                           int8_t actuator_id,
                           float coefficient,
                           float mass,
                           Matrix3f inertia_tensor,
                           Vector3f center_of_mass)
{
  manipulator->addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation, actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void MANAGER::addTool(Manipulator *manipulator,
                      Name my_name,
                      Name parent_name,
                      Vector3f relative_position,
                      Matrix3f relative_orientation,
                      int8_t tool_id,
                      float coefficient,
                      float mass,
                      Matrix3f inertia_tensor,
                      Vector3f center_of_mass)
{
  manipulator->addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void MANAGER::addComponentChild(Manipulator *manipulator, Name my_name, Name child_name)
{
  manipulator->addComponentChild(my_name, child_name);
}

void MANAGER::checkManipulatorSetting(Manipulator *manipulator)
{
  manipulator->checkManipulatorSetting();
}

void MANAGER::setWorldPose(Manipulator *manipulator, Pose world_pose)
{
  manipulator->setWorldPose(world_pose);
}

void MANAGER::setWorldPosition(Manipulator *manipulator, Vector3f world_position)
{
  manipulator->setWorldPosition(world_position);
}

void MANAGER::setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation)
{
  manipulator->setWorldOrientation(world_orientation);
}

void MANAGER::setWorldState(Manipulator *manipulator, State world_state)
{
  manipulator->setWorldState(world_state);
}

void MANAGER::setWorldVelocity(Manipulator *manipulator, VectorXf world_velocity)
{
  manipulator->setWorldVelocity(world_velocity);
}

void MANAGER::setWorldAcceleration(Manipulator *manipulator, VectorXf world_acceleration)
{
  manipulator->setWorldAcceleration(world_acceleration);
}

void MANAGER::setComponent(Manipulator *manipulator, Name name, Component component)
{
  manipulator->setComponent(name, component);
}

void MANAGER::setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world)
{
  manipulator->setComponentPoseToWorld(name, pose_to_world);
}

void MANAGER::setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world)
{
  manipulator->setComponentPositionToWorld(name, position_to_world);
}

void MANAGER::setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd)
{
  manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void MANAGER::setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world)
{
  manipulator->setComponentStateToWorld(name, state_to_world);
}

void MANAGER::setComponentVelocityToWorld(Manipulator *manipulator, Name name, VectorXf velocity)
{
  manipulator->setComponentVelocityToWorld(name, velocity);
}

void MANAGER::setComponentAccelerationToWorld(Manipulator *manipulator, Name name, VectorXf accelaration)
{
  manipulator->setComponentAccelerationToWorld(name, accelaration);
}

void MANAGER::setComponentJointAngle(Manipulator *manipulator, Name name, float angle)
{
  manipulator->setComponentJointAngle(name, angle);
}

void MANAGER::setAllActiveJointAngle(Manipulator *manipulator, std::vector<float> angle_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for(it = manipulator->getIteratorBegin(); it != manipulator->getIteratorEnd(); it++)
  {
    if(manipulator->getComponentJointId(it->first) != -1)
    {
      manipulator->setComponentJointAngle(it->first, angle_vector.at(index));
      index++;
    }
  }
}

void MANAGER::setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity)
{
  manipulator->setComponentJointVelocity(name, angular_velocity);
}

void MANAGER::setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration)
{
  manipulator->setComponentJointAcceleration(name, angular_acceleration);
}

void MANAGER::setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off)
{
  manipulator->setComponentToolOnOff(name, on_off);
}

void MANAGER::setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value)
{
  manipulator->setComponentToolValue(name, actuator_value);
}

int8_t MANAGER::getDOF(Manipulator *manipulator)
{
  return manipulator->getDOF();
}

int8_t MANAGER::getComponentSize(Manipulator *manipulator)
{
  return manipulator->getComponentSize();
}

Name MANAGER::getWorldName(Manipulator *manipulator)
{
  return manipulator->getWorldName();
}

Name MANAGER::getWorldChildName(Manipulator *manipulator)
{
  return manipulator->getWorldChildName();
}

Pose MANAGER::getWorldPose(Manipulator *manipulator)
{
  return manipulator->getWorldPose();
}

Vector3f MANAGER::getWorldPosition(Manipulator *manipulator)
{
  return manipulator->getWorldPosition();
}

Matrix3f MANAGER::getWorldOrientation(Manipulator *manipulator)
{
  return manipulator->getWorldOrientation();
}

State MANAGER::getWorldState(Manipulator *manipulator)
{
  return manipulator->getWorldState();
}

VectorXf MANAGER::getWorldVelocity(Manipulator *manipulator)
{
  return manipulator->getWorldVelocity();
}

VectorXf MANAGER::getWorldAcceleration(Manipulator *manipulator)
{
  return manipulator->getWorldAcceleration();
}

std::map<Name, Component> MANAGER::getAllComponent(Manipulator *manipulator)
{
  return manipulator->getAllComponent();
}

std::map<Name, Component>::iterator MANAGER::getIteratorBegin(Manipulator *manipulator)
{
  return manipulator->getIteratorBegin();
}

std::map<Name, Component>::iterator MANAGER::getIteratorEnd(Manipulator *manipulator)
{
  return manipulator->getIteratorEnd();
}

Component MANAGER::getComponent(Manipulator *manipulator, Name name)
{
  return manipulator->getComponent(name);
}

Name MANAGER::getComponentParentName(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentParentName(name);
}

std::vector<Name> MANAGER::getComponentChildName(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentChildName(name);
}

Pose MANAGER::getComponentPoseToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPoseToWorld(name);
}

Vector3f MANAGER::getComponentPositionToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPositionToWorld(name);
}

Matrix3f MANAGER::getComponentOrientationToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentOrientationToWorld(name);
}

State MANAGER::getComponentStateToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentStateToWorld(name);
}

VectorXf MANAGER::getComponentVelocityToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentVelocityToWorld(name);
}

VectorXf MANAGER::getComponentAccelerationToWorld(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentAccelerationToWorld(name);
}

Pose MANAGER::getComponentRelativePoseToParent(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePoseToParent(name);
}

Vector3f MANAGER::getComponentRelativePositionToParent(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePositionToParent(name);
}

Matrix3f MANAGER::getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativeOrientationToParent(name);
}

Joint MANAGER::getComponentJoint(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJoint(name);
}

int8_t MANAGER::getComponentJointId(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointId(name);
}

float MANAGER::getComponentJointCoefficient(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointCoefficient(name);
}

Vector3f MANAGER::getComponentJointAxis(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAxis(name);
}

float MANAGER::getComponentJointAngle(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAngle(name);
}

std::vector<float> MANAGER::getAllJointAngle(Manipulator *manipulator)
{
  std::vector<float> result_vector;
  std::map<Name, Component>::iterator it;
  for(it = manipulator->getIteratorBegin(); it != manipulator->getIteratorEnd(); it++)
  {
    if(manipulator->getComponentToolId(it->first) < 0)
    {
      result_vector.push_back(manipulator->getComponentJointAngle(it->first));
    }
  }
  return result_vector;
}

std::vector<float> MANAGER::getAllActiveJointAngle(Manipulator *manipulator)
{
  std::vector<float> result_vector;
  std::map<Name, Component>::iterator it;

  for(it = manipulator->getIteratorBegin(); it != manipulator->getIteratorEnd(); it++)
  {
    if(manipulator->getComponentJointId(it->first) != -1)
    {
      result_vector.push_back(manipulator->getComponentJointAngle(it->first));
    }
  }
  return result_vector;
}

float MANAGER::getComponentJointVelocity(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointVelocity(name);
}

float MANAGER::getComponentJointAcceleration(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAcceleration(name);
}

Tool MANAGER::getComponentTool(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentTool(name);
}

int8_t MANAGER::getComponentToolId(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolId(name);
}

float MANAGER::getComponentToolCoefficient(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolCoefficient(name);
}

bool MANAGER::getComponentToolOnOff(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolOnOff(name);
}

float MANAGER::getComponentToolValue(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolValue(name);
}

float MANAGER::getComponentMass(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentMass(name);
}

Matrix3f MANAGER::getComponentInertiaTensor(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentInertiaTensor(name);
}

Vector3f MANAGER::getComponentCenterOfMass(Manipulator *manipulator, Name name)
{
  return manipulator->getComponentCenterOfMass(name);  
}