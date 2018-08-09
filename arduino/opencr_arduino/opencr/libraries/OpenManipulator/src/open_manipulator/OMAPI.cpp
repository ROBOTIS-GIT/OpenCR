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

////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

//manager
void MANAGER::addWorld(OM_MANAGER::Manipulator *manipulator,
                       Name world_name,
                       Name child_name,
                       Vector3f world_position,
                       Matrix3f world_orientation)
{
  manipulator->addWorld(world_name, child_name, world_position, world_orientation);
}

void MANAGER::addComponent(OM_MANAGER::Manipulator *manipulator,
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

void MANAGER::addTool(OM_MANAGER::Manipulator *manipulator,
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

void MANAGER::addComponentChild(OM_MANAGER::Manipulator *manipulator, Name my_name, Name child_name)
{
  manipulator->addComponentChild(my_name, child_name);
}

void MANAGER::checkManipulatorSetting(OM_MANAGER::Manipulator *manipulator)
{
  manipulator->checkManipulatorSetting();
}

void MANAGER::setWorldPose(OM_MANAGER::Manipulator *manipulator, Pose world_pose)
{
  manipulator->setWorldPose(world_pose);
}

void MANAGER::setWorldPosition(OM_MANAGER::Manipulator *manipulator, Vector3f world_position)
{
  manipulator->setWorldPosition(world_position);
}

void MANAGER::setWorldOrientation(OM_MANAGER::Manipulator *manipulator, Matrix3f world_orientation)
{
  manipulator->setWorldOrientation(world_orientation);
}

void MANAGER::setWorldState(OM_MANAGER::Manipulator *manipulator, State world_state)
{
  manipulator->setWorldState(world_state);
}

void MANAGER::setWorldVelocity(OM_MANAGER::Manipulator *manipulator, VectorXf world_velocity)
{
  manipulator->setWorldVelocity(world_velocity);
}

void MANAGER::setWorldAcceleration(OM_MANAGER::Manipulator *manipulator, VectorXf world_acceleration)
{
  manipulator->setWorldAcceleration(world_acceleration);
}

void MANAGER::setComponent(OM_MANAGER::Manipulator *manipulator, Name name, Component component)
{
  manipulator->setComponent(name, component);
}

void MANAGER::setComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Pose pose_to_world)
{
  manipulator->setComponentPoseToWorld(name, pose_to_world);
}

void MANAGER::setComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Vector3f position_to_world)
{
  manipulator->setComponentPositionToWorld(name, position_to_world);
}

void MANAGER::setComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd)
{
  manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void MANAGER::setComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name, State state_to_world)
{
  manipulator->setComponentStateToWorld(name, state_to_world);
}

void MANAGER::setComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf velocity)
{
  manipulator->setComponentVelocityToWorld(name, velocity);
}

void MANAGER::setComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf accelaration)
{
  manipulator->setComponentAccelerationToWorld(name, accelaration);
}

void MANAGER::setComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name, float angle)
{
  manipulator->setComponentJointAngle(name, angle);
}

void MANAGER::setAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator, std::vector<float> angle_vector)
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

void MANAGER::setComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name, float angular_velocity)
{
  manipulator->setComponentJointVelocity(name, angular_velocity);
}

void MANAGER::setComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name, float angular_acceleration)
{
  manipulator->setComponentJointAcceleration(name, angular_acceleration);
}

void MANAGER::setComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name, bool on_off)
{
  manipulator->setComponentToolOnOff(name, on_off);
}

void MANAGER::setComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name, float actuator_value)
{
  manipulator->setComponentToolValue(name, actuator_value);
}

int8_t MANAGER::getDOF(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getDOF();
}

int8_t MANAGER::getComponentSize(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getComponentSize();
}

Name MANAGER::getWorldName(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldName();
}

Name MANAGER::getWorldChildName(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldChildName();
}

Pose MANAGER::getWorldPose(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPose();
}

Vector3f MANAGER::getWorldPosition(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPosition();
}

Matrix3f MANAGER::getWorldOrientation(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldOrientation();
}

State MANAGER::getWorldState(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldState();
}

VectorXf MANAGER::getWorldVelocity(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldVelocity();
}

VectorXf MANAGER::getWorldAcceleration(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldAcceleration();
}

std::map<Name, Component> MANAGER::getAllComponent(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getAllComponent();
}

std::map<Name, Component>::iterator MANAGER::getIteratorBegin(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorBegin();
}

std::map<Name, Component>::iterator MANAGER::getIteratorEnd(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorEnd();
}

Component MANAGER::getComponent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponent(name);
}

Name MANAGER::getComponentParentName(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentParentName(name);
}

std::vector<Name> MANAGER::getComponentChildName(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentChildName(name);
}

Pose MANAGER::getComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPoseToWorld(name);
}

Vector3f MANAGER::getComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPositionToWorld(name);
}

Matrix3f MANAGER::getComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentOrientationToWorld(name);
}

State MANAGER::getComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentStateToWorld(name);
}

VectorXf MANAGER::getComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentVelocityToWorld(name);
}

VectorXf MANAGER::getComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentAccelerationToWorld(name);
}

Pose MANAGER::getComponentRelativePoseToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePoseToParent(name);
}

Vector3f MANAGER::getComponentRelativePositionToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePositionToParent(name);
}

Matrix3f MANAGER::getComponentRelativeOrientationToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativeOrientationToParent(name);
}

Joint MANAGER::getComponentJoint(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJoint(name);
}

int8_t MANAGER::getComponentJointId(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointId(name);
}

float MANAGER::getComponentJointCoefficient(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointCoefficient(name);
}

Vector3f MANAGER::getComponentJointAxis(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAxis(name);
}

float MANAGER::getComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAngle(name);
}

std::vector<float> MANAGER::getAllJointAngle(OM_MANAGER::Manipulator *manipulator)
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

std::vector<float> MANAGER::getAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator)
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

float MANAGER::getComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointVelocity(name);
}

float MANAGER::getComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAcceleration(name);
}

Tool MANAGER::getComponentTool(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentTool(name);
}

int8_t MANAGER::getComponentToolId(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolId(name);
}

float MANAGER::getComponentToolCoefficient(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolCoefficient(name);
}

bool MANAGER::getComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolOnOff(name);
}

float MANAGER::getComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolValue(name);
}

float MANAGER::getComponentMass(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentMass(name);
}

Matrix3f MANAGER::getComponentInertiaTensor(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentInertiaTensor(name);
}

Vector3f MANAGER::getComponentCenterOfMass(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentCenterOfMass(name);  
}

void KINEMATICS::connectJacobian(MatrixXf (*fp)(OM_MANAGER::Manipulator *, Name)){ KINEMATICS::jacobian = fp; }
// void KINEMATICS::connectForward(void (*fp)(OM_MANAGER::Manipulator *manipulator)) { forward = fp; }
void KINEMATICS::connectForward(void (*fp)(OM_MANAGER::Manipulator *, Name)) { KINEMATICS::forward = fp; }
void KINEMATICS::connectInverse(std::vector<float> (*fp)(OM_MANAGER::Manipulator *, Name, Pose)) { KINEMATICS::inverse = fp; }

MatrixXf KINEMATICS::getJacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name){ KINEMATICS::jacobian(manipulator, tool_name); }
void KINEMATICS::solveForward(OM_MANAGER::Manipulator *manipulator, Name component_name){ KINEMATICS::forward(manipulator, component_name); }
std::vector<float> KINEMATICS::solveInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose){ KINEMATICS::inverse(manipulator, tool_name, target_pose); }