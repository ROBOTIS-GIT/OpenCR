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

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../include/open_manipulator/OMManager.h"

using namespace Eigen;

///////////////////////////*initialize function*/////////////////////////////
void Manipulator::addWorld(Name world_name,
                           Name child_name,
                           Vector3f world_position,
                           Matrix3f world_orientation)
{
  world_.name = world_name;
  world_.child = child_name;
  world_.pose.position = world_position;
  world_.pose.orientation = world_orientation;
  world_.origin.velocity = VectorXf::Zero(3);
  world_.origin.acceleration = VectorXf::Zero(3);
}

void Manipulator::addComponent(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Vector3f relative_position,
                               Matrix3f relative_orientation,
                               int8_t joint_actuator_id,
                               Vector3f axis_of_rotation,
                               float mass,
                               Matrix3f inertia_tensor,
                               Vector3f center_of_mass)
{
  if (joint_actuator_id != -1)
    dof_++;

  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.child.push_back(child_name);
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Vector3f::Zero();
  temp_component.pose_to_world.orientation = Matrix3f::Identity(3, 3);
  temp_component.origin.velocity = VectorXf::Zero(3);
  temp_component.origin.acceleration = VectorXf::Zero(3);
  temp_component.joint.id = joint_actuator_id;
  temp_component.joint.axis = axis_of_rotation;
  temp_component.joint.angle = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = -1;
  temp_component.tool.on_off = false;
  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::addComponentChild(Name my_name, Name child_name)
{
  component_.at(my_name).child.push_back(child_name);
}

void Manipulator::addTool(Name my_name,
                          Name parent_name,
                          Vector3f relative_position,
                          Matrix3f relative_orientation,
                          int8_t tool_id,
                          float mass,
                          Matrix3f inertia_tensor,
                          Vector3f center_of_mass)
{
  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Vector3f::Zero();
  temp_component.pose_to_world.orientation = Matrix3f::Identity(3, 3);
  temp_component.origin.velocity = VectorXf::Zero(3);
  temp_component.origin.acceleration = VectorXf::Zero(3);
  temp_component.joint.id = -1;
  temp_component.joint.axis = Vector3f::Zero();
  temp_component.joint.angle = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = tool_id;
  temp_component.tool.on_off = false;
  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::checkManipulatorSetting()
{
  USB.println("Degree of freedom : " + String(dof_));
  USB.println("Size of Components : " + String(component_.size()));

  USB.println("\n<Configuration of world>");
  USB.println("Name : " + String(world_.name));
  USB.println("Child name : " + String(world_.child));
  USB.println("Position : ");
  PRINT::VECTOR(world_.pose.position);
  USB.println("Orientation : ");
  PRINT::MATRIX(world_.pose.orientation);
  USB.println("Velocity : ");
  PRINT::VECTOR(world_.origin.velocity);
  USB.println("Acceleration : ");
  PRINT::VECTOR(world_.origin.acceleration);

  USB.println("\n<Configuration of components>");

  for (std::map<Name, Component>::iterator it = component_.begin(); it != component_.end(); it++)
  {
    USB.println("Name : " + String(it->first) + " -----------------------------------------------");
    USB.println("Parent : " + String(component_[it->first].parent));

    for (std::vector<Name>::size_type index = 0; index < component_[it->first].child.size(); index++)
      USB.println("Child  : " + String(component_[it->first].child[index]));

    USB.println("\nRelative to parent");
    USB.println(" Position : ");
    PRINT::VECTOR(component_[it->first].relative_to_parent.position);
    USB.println(" Orientation : ");
    PRINT::MATRIX(component_[it->first].relative_to_parent.orientation);

    USB.println("\nPose to world");
    USB.println(" Position : ");
    PRINT::VECTOR(component_[it->first].pose_to_world.position);
    USB.println(" Orientation : ");
    PRINT::MATRIX(component_[it->first].pose_to_world.orientation);

    USB.println("\nState to origin");
    USB.println(" Velocity : ");
    PRINT::VECTOR(component_[it->first].origin.velocity);
    USB.println(" Acceleration : ");
    PRINT::MATRIX(component_[it->first].origin.acceleration);

    USB.println("\nJoint");
    USB.println(" ID : " + String(component_[it->first].joint.id));
    USB.println(" Axis : ");
    PRINT::VECTOR(component_[it->first].joint.axis);
    USB.print(" Angle : ");+ USB.println(component_[it->first].joint.angle);
    USB.print(" Velocity : "); + USB.println(component_[it->first].joint.velocity);
    USB.print(" Acceleration : "); + USB.println(component_[it->first].joint.acceleration);

    USB.println("\nTool");
    USB.println(" ID : " + String(component_[it->first].tool.id));
    USB.println(" OnOff : " + String(component_[it->first].tool.on_off));
    USB.println(" Value : "); USB.println(component_[it->first].tool.value);

    USB.println("\nInertia");
    USB.print(" Mass : "); USB.println(component_[it->first].inertia.mass);
    USB.println(" Inertia tensor : ");
    PRINT::MATRIX(component_[it->first].inertia.inertia_tensor);
    USB.println(" Center of mass : ");
    PRINT::VECTOR(component_[it->first].inertia.center_of_mass);
    USB.println();
  }
}
/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setWorldPose(Pose world_pose)
{
  world_.pose = world_pose;
}

void Manipulator::setWorldPosition(Vector3f world_position)
{
  world_.pose.position = world_position;
}

void Manipulator::setWorldOrientation(Matrix3f world_orientation)
{
  world_.pose.orientation = world_orientation;
}

void Manipulator::setWorldState(State world_state)
{
  world_.origin = world_state;
}

void Manipulator::setWorldVelocity(VectorXf world_velocity)
{
  world_.origin.velocity = world_velocity;
}

void Manipulator::setWorldAcceleration(VectorXf world_acceleration)
{
  world_.origin.acceleration = world_acceleration;
}

void Manipulator::setComponent(Name name, Component component, bool *error)
{
  if (component_.find(name) != component_.end())
  {
    component_.insert(std::make_pair(name, component));
    *error = false;
  }
  else
  {
    *error = true;
  }
}

void Manipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world = pose_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.position = position_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.orientation = orientation_to_wolrd;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentStateToWorld(Name name, State state_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).origin = state_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentVelocityToWorld(Name name, VectorXf velocity)
{
  if (velocity.size() != 6)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).origin.velocity = velocity;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentAccelerationToWorld(Name name, VectorXf acceleration)
{
  if (acceleration.size() != 6)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).origin.acceleration = acceleration;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointAngle(Name name, float angle)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.angle = angle;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointVelocity(Name name, float angular_velocity)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.velocity = angular_velocity;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointAcceleration(Name name, float angular_acceleration)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.acceleration = angular_acceleration;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentToolOnOff(Name name, bool on_off)
{
  if (component_.at(name).tool.id > 0)
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.on_off = on_off;
    }
    else
    {
      //error
    }
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentToolValue(Name name, float value)
{
  if (component_.at(name).tool.id > 0)
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.value = value;
    }
    else
    {
      //error
    }
  }
  else
  {
    //error
  }
}

///////////////////////////////Get function//////////////////////////////////

int8_t Manipulator::getDOF()
{
  return dof_;
}

int8_t Manipulator::getComponentSize()
{
  return component_.size();
}

Name Manipulator::getWorldName()
{
  return world_.name;
}

Name Manipulator::getWorldChildName()
{
  return world_.child;
}

Pose Manipulator::getWorldPose()
{
  return world_.pose;
}

Vector3f Manipulator::getWorldPosition()
{
  return world_.pose.position;
}

Matrix3f Manipulator::getWorldOrientation()
{
  return world_.pose.orientation;
}

State Manipulator::getWorldState()
{
  return world_.origin;
}

VectorXf Manipulator::getWorldVelocity()
{
  return world_.origin.velocity;
}

VectorXf Manipulator::getWorldAcceleration()
{
  return world_.origin.acceleration;
}

std::map<Name, Component> Manipulator::getAllComponent()
{
  return component_;
}

Component Manipulator::getComponent(Name name)
{
  return component_.at(name);
}

Name Manipulator::getComponentParentName(Name name)
{
  return component_.at(name).parent;
}

std::vector<Name> Manipulator::getComponentChildName(Name name)
{
  return component_.at(name).child;
}

Pose Manipulator::getComponentPoseToWorld(Name name)
{
  return component_.at(name).pose_to_world;
}

Vector3f Manipulator::getComponentPositionToWorld(Name name)
{
  return component_.at(name).pose_to_world.position;
}

Matrix3f Manipulator::getComponentOrientationToWorld(Name name)
{
  return component_.at(name).pose_to_world.orientation;
}

State Manipulator::getComponentStateToWorld(Name name)
{
  return component_.at(name).origin;
}

VectorXf Manipulator::getComponentVelocityToWorld(Name name)
{
  return component_.at(name).origin.velocity;
}

VectorXf Manipulator::getComponentAccelerationToWorld(Name name)
{
  return component_.at(name).origin.acceleration;
}

Pose Manipulator::getComponentRelativePoseToParent(Name name)
{
  return component_.at(name).relative_to_parent;
}

Vector3f Manipulator::getComponentRelativePositionToParent(Name name)
{
  return component_.at(name).relative_to_parent.position;
}

Matrix3f Manipulator::getComponentRelativeOrientationToParent(Name name)
{
  return component_.at(name).relative_to_parent.orientation;
}

Joint Manipulator::getComponentJoint(Name name)
{
  return component_.at(name).joint;
}

int8_t Manipulator::getComponentJointId(Name name)
{
  return component_.at(name).joint.id;
}

Vector3f Manipulator::getComponentJointAxis(Name name)
{
  return component_.at(name).joint.axis;
}

float Manipulator::getComponentJointAngle(Name name)
{
  return component_.at(name).joint.angle;
}

float Manipulator::getComponentJointVelocity(Name name)
{
  return component_.at(name).joint.velocity;
}

float Manipulator::getComponentJointAcceleration(Name name)
{
  return component_.at(name).joint.acceleration;
}

Tool Manipulator::getComponentTool(Name name)
{
  return component_.at(name).tool;
}

int8_t Manipulator::getComponentToolId(Name name)
{
  return component_.at(name).tool.id;
}

bool Manipulator::getComponentToolOnOff(Name name)
{
  return component_.at(name).tool.on_off;
}

float Manipulator::getComponentToolValue(Name name)
{
  return component_.at(name).tool.value;
}

float Manipulator::getComponentMass(Name name)
{
  return component_.at(name).inertia.mass;
}

Matrix3f Manipulator::getComponentInertiaTensor(Name name)
{
  return component_.at(name).inertia.inertia_tensor;
}

Vector3f Manipulator::getComponentCenterOfMass(Name name)
{
  return component_.at(name).inertia.center_of_mass;
}
