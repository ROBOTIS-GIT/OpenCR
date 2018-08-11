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

void OpenManipulator::addWorld(OM_MANAGER::Manipulator *manipulator,
                       Name world_name,
                       Name child_name,
                       Vector3f world_position,
                       Matrix3f world_orientation)
{
  manipulator->addWorld(world_name, child_name, world_position, world_orientation);
}

void OpenManipulator::addComponent(OM_MANAGER::Manipulator *manipulator,
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

void OpenManipulator::addTool(OM_MANAGER::Manipulator *manipulator,
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

void OpenManipulator::addComponentChild(OM_MANAGER::Manipulator *manipulator, Name my_name, Name child_name)
{
  manipulator->addComponentChild(my_name, child_name);
}

void OpenManipulator::checkManipulatorSetting(OM_MANAGER::Manipulator *manipulator)
{
  manipulator->checkManipulatorSetting();
}

void OpenManipulator::setWorldPose(OM_MANAGER::Manipulator *manipulator, Pose world_pose)
{
  manipulator->setWorldPose(world_pose);
}

void OpenManipulator::setWorldPosition(OM_MANAGER::Manipulator *manipulator, Vector3f world_position)
{
  manipulator->setWorldPosition(world_position);
}

void OpenManipulator::setWorldOrientation(OM_MANAGER::Manipulator *manipulator, Matrix3f world_orientation)
{
  manipulator->setWorldOrientation(world_orientation);
}

void OpenManipulator::setWorldState(OM_MANAGER::Manipulator *manipulator, State world_state)
{
  manipulator->setWorldState(world_state);
}

void OpenManipulator::setWorldVelocity(OM_MANAGER::Manipulator *manipulator, VectorXf world_velocity)
{
  manipulator->setWorldVelocity(world_velocity);
}

void OpenManipulator::setWorldAcceleration(OM_MANAGER::Manipulator *manipulator, VectorXf world_acceleration)
{
  manipulator->setWorldAcceleration(world_acceleration);
}

void OpenManipulator::setComponent(OM_MANAGER::Manipulator *manipulator, Name name, Component component)
{
  manipulator->setComponent(name, component);
}

void OpenManipulator::setComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Pose pose_to_world)
{
  manipulator->setComponentPoseToWorld(name, pose_to_world);
}

void OpenManipulator::setComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Vector3f position_to_world)
{
  manipulator->setComponentPositionToWorld(name, position_to_world);
}

void OpenManipulator::setComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd)
{
  manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void OpenManipulator::setComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name, State state_to_world)
{
  manipulator->setComponentStateToWorld(name, state_to_world);
}

void OpenManipulator::setComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf velocity)
{
  manipulator->setComponentVelocityToWorld(name, velocity);
}

void OpenManipulator::setComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf accelaration)
{
  manipulator->setComponentAccelerationToWorld(name, accelaration);
}

void OpenManipulator::setComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name, float angle)
{
  manipulator->setComponentJointAngle(name, angle);
}

void OpenManipulator::setAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator, std::vector<float> angle_vector)
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

void OpenManipulator::setComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name, float angular_velocity)
{
  manipulator->setComponentJointVelocity(name, angular_velocity);
}

void OpenManipulator::setComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name, float angular_acceleration)
{
  manipulator->setComponentJointAcceleration(name, angular_acceleration);
}

void OpenManipulator::setComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name, bool on_off)
{
  manipulator->setComponentToolOnOff(name, on_off);
}

void OpenManipulator::setComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name, float actuator_value)
{
  manipulator->setComponentToolValue(name, actuator_value);
}

int8_t OpenManipulator::getDOF(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getDOF();
}

int8_t OpenManipulator::getComponentSize(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getComponentSize();
}

Name OpenManipulator::getWorldName(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldName();
}

Name OpenManipulator::getWorldChildName(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldChildName();
}

Pose OpenManipulator::getWorldPose(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPose();
}

Vector3f OpenManipulator::getWorldPosition(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPosition();
}

Matrix3f OpenManipulator::getWorldOrientation(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldOrientation();
}

State OpenManipulator::getWorldState(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldState();
}

VectorXf OpenManipulator::getWorldVelocity(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldVelocity();
}

VectorXf OpenManipulator::getWorldAcceleration(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldAcceleration();
}

std::map<Name, Component> OpenManipulator::getAllComponent(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getAllComponent();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorBegin(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorBegin();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorEnd(OM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorEnd();
}

Component OpenManipulator::getComponent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponent(name);
}

Name OpenManipulator::getComponentParentName(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentParentName(name);
}

std::vector<Name> OpenManipulator::getComponentChildName(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentChildName(name);
}

Pose OpenManipulator::getComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPoseToWorld(name);
}

Vector3f OpenManipulator::getComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPositionToWorld(name);
}

Matrix3f OpenManipulator::getComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentOrientationToWorld(name);
}

State OpenManipulator::getComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentStateToWorld(name);
}

VectorXf OpenManipulator::getComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentVelocityToWorld(name);
}

VectorXf OpenManipulator::getComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentAccelerationToWorld(name);
}

Pose OpenManipulator::getComponentRelativePoseToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePoseToParent(name);
}

Vector3f OpenManipulator::getComponentRelativePositionToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePositionToParent(name);
}

Matrix3f OpenManipulator::getComponentRelativeOrientationToParent(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativeOrientationToParent(name);
}

Joint OpenManipulator::getComponentJoint(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJoint(name);
}

int8_t OpenManipulator::getComponentJointId(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointId(name);
}

float OpenManipulator::getComponentJointCoefficient(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointCoefficient(name);
}

Vector3f OpenManipulator::getComponentJointAxis(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAxis(name);
}

float OpenManipulator::getComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAngle(name);
}

std::vector<float> OpenManipulator::getAllJointAngle(OM_MANAGER::Manipulator *manipulator)
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

std::vector<float> OpenManipulator::getAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator)
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

float OpenManipulator::getComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointVelocity(name);
}

float OpenManipulator::getComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAcceleration(name);
}

Tool OpenManipulator::getComponentTool(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentTool(name);
}

int8_t OpenManipulator::getComponentToolId(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolId(name);
}

float OpenManipulator::getComponentToolCoefficient(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolCoefficient(name);
}

bool OpenManipulator::getComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolOnOff(name);
}

float OpenManipulator::getComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolValue(name);
}

float OpenManipulator::getComponentMass(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentMass(name);
}

Matrix3f OpenManipulator::getComponentInertiaTensor(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentInertiaTensor(name);
}

Vector3f OpenManipulator::getComponentCenterOfMass(OM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentCenterOfMass(name);  
}