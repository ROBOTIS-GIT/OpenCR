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

#include "../../../include/open_manipulator/OMAPI.h"


////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////
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

  void MANAGER::addWorld(Manipulator *manipulator, Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error)
  {
    manipulator->addWorld(world_name, child_name, world_position, world_orientation, error);
  }

  void MANAGER::addComponent(Manipulator *manipulator, Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error)
  {
    manipulator->addComponent(me_name, parent_name, child_name, relative_position, relative_orientation, actuator_id, axis_of_rotation, mass, inertia_tensor, center_of_mass, *error);
  }

  void MANAGER::addTool(Manipulator *manipulator, Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1,
              float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero())
  {
    manipulator->addTool(me_name, parent_name, relative_position, relative_orientation, tool_id, mass, inertia_tensor, Vector3f center_of_mass);
  }

  void MANAGER::addComponentChild(Manipulator *manipulator, Name me_name, NAME child_name, bool *error)
  {
    manipulator->addComponentChild(me_name, child_name, error);
  }

  void MANAGER::checkManipulatorSetting(Manipulator *manipulator, bool *error)
  {
    manipulator->checkManipulatorSetting(error);
  }

  void MANAGER::setWorldPose(Manipulator *manipulator, Pose world_pose, bool *error)
  {
    manipulator->setWorldPose();
  }

  void MANAGER::setWorldPosition(Manipulator *manipulator, Vector3f world_position, bool *error)
  {
    manipulator->setWorldPosition(world_position, error);
  }

  void MANAGER::setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation, bool *error)
  {
    manipulator->setWorldOrientation(world_orientation, error);
  }

  void MANAGER::setWorldState(Manipulator *manipulator, State world_state, bool *error)
  {
    manipulator->setWorldState(world_state, error);
  }

  void MANAGER::setWorldVelocity(Manipulator *manipulator, Vector6f world_velocity, bool *error)
  {
    manipulator->setWorldVelocity(world_velocity, error);
  }

  void MANAGER::setWorldAcceleration(Manipulator *manipulator, Vector6f world_acceleration, bool *error)
  {
    manipulator->setWorldAcceleration(world_acceleration, error);
  }
  
  void MANAGER::setComponent(Manipulator *manipulator, Name name, Component component, bool *error)
  {
    manipulator->setComponent(name, component, error);
  }

  void MANAGER::setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world, bool *error)
  {
    manipulator->setComponentPoseToWorld(name, pose_to_world, error);
  }

  void MANAGER::setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world, bool *error)
  {
    manipulator->setComponentPositionToWorld(name, position_to_world, error);
  }

  void MANAGER::setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd, bool *error)
  {
    manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd, error);
  }

  void MANAGER::setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world, bool *error)
  {
    manipulator->setComponentStateToWorld(name, state_to_world, error);
  }

  void MANAGER::setComponentVelocityToWorld(Manipulator *manipulator, Name name, Vector6f velocity, bool *error)
  {
    manipulator->setComponentVelocityToWorld(name, velocity, error);
  }

  void MANAGER::setComponentAccelerationToWorld(Manipulator *manipulator, Name name, Vector6f accelaration, bool *error)
  {
    manipulator->setComponentAccelerationToWorld(name, accelaration, error);
  }

  void MANAGER::setComponentJointAngle(Manipulator *manipulator, Name name, float angle, bool *error)
  {
    manipulator->setComponentJointAngle(name, angle, error);
  }

  void MANAGER::setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity, bool *error)
  {
    manipulator->setComponentJointVelocity(name, angular_velocity, error);
  }

  void MANAGER::setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration, bool *error)
  {
    manipulator->setComponentJointAcceleration(name, angular_acceleration, error);
  }

  void MANAGER::setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off, bool *error)
  {
    manipulator->setComponentToolOnOff(name, on_off, error);
  }

  void MANAGER::setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value, bool *error)
  {
    manipulator->setComponentToolValue(name, actuator_value, error);
  }

  int MANAGER::getDOF(Manipulator *manipulator, bool *error)
  {
    return manipulator->getDOF(error);
  }

  int8_t MANAGER::getComponentSize(Manipulator *manipulator, bool *error)
  {
    return manipulator->getComponentSize(error);
  }

  Name MANAGER::getWorldName((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldName(error);
  }

  Name MANAGER::getWorldChildName((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldChildName(error);
  }

  Pose MANAGER::getWorldPose((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldPose(error);
  }

  Vector3f MANAGER::getWorldPosition((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldPosition(error);
  }

  Matrix3f MANAGER::getWorldOrientation((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldOrientation(error);
  }

  State MANAGER::getWorldState((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldState(error);
  }

  Vector6f MANAGER::getWorldVelocity((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldVelocity(error);
  }

  Vector6f MANAGER::getWorldAcceleration((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldAcceleration(error);
  }

  map<Name, Component> MANAGER::getAllComponent((Manipulator *manipulator, bool *error)
  {
    return manipulator->getAllComponent(error);
  }

  Component MANAGER::getComponent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponent(name, error);
  }

  Name MANAGER::getComponentParentName(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentParentName(name, error);
  }

  vector<NAME> MANAGER::getComponentChildName(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentChildName(name, error);
  }

  Pose MANAGER::getComponentPoseToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentPoseToWorld(name, error);
  }

  Vector3f MANAGER::getComponentPositionToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentPositionToWorld(name, error);
  }

  Matrix3f MANAGER::getComponentOrientationToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentOrientationToWorld(name, error);
  }

  State MANAGER::getComponentStateToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentStateToWorld(name, error);
  }

  Vector6f MANAGER::getComponentVelocityToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentVelocityToWorld(name, error);
  }

  Vector6f MANAGER::getComponentAccelerationToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentAccelerationToWorld(name, error);
  }

  Pose MANAGER::getComponentRelativePoseToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativePoseToParent(name, error);
  }

  Vector3f MANAGER::getComponentRelativePositionToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativePositionToParent(name, error);
  }

  Matrix3f MANAGER::getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativeOrientationToParent(name, error);
  }

  Joint MANAGER::getComponentJoint(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJoint(name, error);
  }

  int8_t MANAGER::getComponentJointId(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointId(name, error);
  }

  Vector3f MANAGER::getComponentJointAxis(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAxis(name, error);
  }

  float MANAGER::getComponentJointAngle(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAngle(name, error);
  }

  float MANAGER::getComponentJointVelocity(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointVelocity(name, error);
  }

  float MANAGER::getComponentJointAcceleration(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAcceleration(name, error);
  }

  Tool MANAGER::getComponentTool(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentTool(name, error);
  }

  int8_t MANAGER::getComponentToolId(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolId(name, error);
  }

  bool MANAGER::getComponentToolOnOff(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolOnOff(name, error);
  }

  float MANAGER::getComponentToolValue(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolValue(name, error);
  }

  float MANAGER::getComponentMass(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentMass(name, error);
  }

  Matrix3f MANAGER::getComponentInertiaTensor(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentInertiaTensor(name, error);
  }

  Pose MANAGER::getComponentCenterOfMassPose(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassPose(name, error);
  }

  Vector3f MANAGER::getComponentCenterOfMassPosition(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassPosition(name, error);
  }

  Matrix3f MANAGER::getComponentCenterOfMassOrientation(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassOrientation(name, error);
  }