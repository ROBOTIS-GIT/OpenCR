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

#include "../../include/robotis_manipulator/robotis_manipulator_common.h"

using namespace ROBOTIS_MANIPULATOR;

/////////////////////Manipulator class//////////////////////
Manipulator::Manipulator()
    :dof_(0)
{}
///////////////////////////////add function//////////////////////////////////
void Manipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Matrix3d world_orientation)
{
  world_.name = world_name;
  world_.child = child_name;
  world_.pose.kinematic.position = world_position;
  world_.pose.kinematic.orientation = world_orientation;
  world_.pose.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  world_.pose.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  world_.pose.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  world_.pose.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);
}

void Manipulator::addJoint(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Eigen::Vector3d relative_position,
                               Eigen::Matrix3d relative_orientation,
                               Eigen::Vector3d axis_of_rotation,
                               int8_t joint_actuator_id,
                               double max_limit,
                               double min_limit,
                               double coefficient,
                               double mass,
                               Eigen::Matrix3d inertia_tensor,
                               Eigen::Vector3d center_of_mass)
{
  Component temp_component;
  if (joint_actuator_id != -1)
  {
    dof_++;
    temp_component.component_type = ACTIVE_JOINT_COMPONENT;
  }
  else
  {
    temp_component.component_type = PASSIVE_JOINT_COMPONENT;
  }

  temp_component.name.parent = parent_name;
  temp_component.name.child.push_back(child_name);
  temp_component.relative.pose_from_parent.position = relative_position;
  temp_component.relative.pose_from_parent.orientation = relative_orientation;
  temp_component.relative.inertia.mass = mass;
  temp_component.relative.inertia.inertia_tensor = inertia_tensor;
  temp_component.relative.inertia.center_of_mass = center_of_mass;
  temp_component.joint_constant.id = joint_actuator_id;
  temp_component.joint_constant.coefficient = coefficient;
  temp_component.joint_constant.axis = axis_of_rotation;
  temp_component.joint_constant.limit.maximum = max_limit;
  temp_component.joint_constant.limit.minimum = min_limit;

  temp_component.pose_from_world.kinematic.position = Eigen::Vector3d::Zero();
  temp_component.pose_from_world.kinematic.orientation = Eigen::Matrix3d::Identity();
  temp_component.pose_from_world.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  temp_component.joint_value.position = 0.0;
  temp_component.joint_value.velocity = 0.0;
  temp_component.joint_value.effort = 0.0;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::addComponentChild(Name my_name, Name child_name)
{
  component_.at(my_name).name.child.push_back(child_name);
}

void Manipulator::addTool(Name my_name,
                          Name parent_name,
                          Eigen::Vector3d relative_position,
                          Eigen::Matrix3d relative_orientation,
                          int8_t tool_id,
                          double max_limit,
                          double min_limit,
                          double coefficient,
                          double mass,
                          Eigen::Matrix3d inertia_tensor,
                          Eigen::Vector3d center_of_mass)
{
  Component temp_component;

  temp_component.name.parent = parent_name;
  temp_component.name.child.resize(0);
  temp_component.component_type = TOOL_COMPONENT;
  temp_component.relative.pose_from_parent.position = relative_position;
  temp_component.relative.pose_from_parent.orientation = relative_orientation;
  temp_component.relative.inertia.mass = mass;
  temp_component.relative.inertia.inertia_tensor = inertia_tensor;
  temp_component.relative.inertia.center_of_mass = center_of_mass;
  temp_component.joint_constant.id = tool_id;
  temp_component.joint_constant.coefficient = coefficient;
  temp_component.joint_constant.axis = Eigen::Vector3d::Zero();
  temp_component.joint_constant.limit.maximum = max_limit;
  temp_component.joint_constant.limit.minimum = min_limit;

  temp_component.pose_from_world.kinematic.position = Eigen::Vector3d::Zero();
  temp_component.pose_from_world.kinematic.orientation = Eigen::Matrix3d::Identity();
  temp_component.pose_from_world.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.pose_from_world.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  temp_component.joint_value.position = 0.0;
  temp_component.joint_value.velocity = 0.0;
  temp_component.joint_value.effort = 0.0;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::checkManipulatorSetting()
{
  RM_LOG::PRINTLN("----------<Manipulator Description>----------");
  RM_LOG::PRINTLN("<Degree of freedom>\n", dof_);
  RM_LOG::PRINTLN("<Size of Components>\n", component_.size());
  RM_LOG::PRINTLN("");
  RM_LOG::PRINTLN("<Configuration of world>");
  RM_LOG::PRINTLN(" [Name]");
  RM_LOG::PRINT(" -World Name : "); RM_LOG::PRINTLN(STRING(world_.name));
  RM_LOG::PRINT(" -Child Name : "); RM_LOG::PRINTLN(STRING(world_.child));
  RM_LOG::PRINTLN(" [Static Pose]");
  RM_LOG::PRINTLN(" -Position : ");
  RM_LOG::PRINT_VECTOR(world_.pose.kinematic.position);
  RM_LOG::PRINTLN(" -Orientation : ");
  RM_LOG::PRINT_MATRIX(world_.pose.kinematic.orientation);
  RM_LOG::PRINTLN(" [Dynamic Pose]");
  RM_LOG::PRINTLN(" -Linear Velocity : ");
  RM_LOG::PRINT_VECTOR(world_.pose.dynamic.linear.velocity);
  RM_LOG::PRINTLN(" -Linear acceleration : ");
  RM_LOG::PRINT_VECTOR(world_.pose.dynamic.linear.acceleration);
  RM_LOG::PRINTLN(" -Angular Velocity : ");
  RM_LOG::PRINT_VECTOR(world_.pose.dynamic.angular.velocity);
  RM_LOG::PRINTLN(" -Angular acceleration : ");
  RM_LOG::PRINT_VECTOR(world_.pose.dynamic.angular.acceleration);

  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    RM_LOG::PRINTLN("");
    RM_LOG::PRINT("<Configuration of "); RM_LOG::PRINT(STRING(it_component->first)); RM_LOG::PRINTLN(">");
    if(component_.at(it_component->first).component_type == ACTIVE_JOINT_COMPONENT)
      RM_LOG::PRINTLN(" [Component Type]\n  Active Joint");
    else if(component_.at(it_component->first).component_type == PASSIVE_JOINT_COMPONENT)
      RM_LOG::PRINTLN(" [Component Type]\n  Passive Joint");
    else if(component_.at(it_component->first).component_type == TOOL_COMPONENT)
      RM_LOG::PRINTLN(" [Component Type]\n  Tool");
    RM_LOG::PRINTLN(" [Name]");
    RM_LOG::PRINT(" -Parent Name : "); RM_LOG::PRINTLN(STRING(component_.at(it_component->first).name.parent));
    for(uint32_t index = 0; index < component_.at(it_component->first).name.child.size(); index++)
    {
      RM_LOG::PRINT(" -Child Name",index+1,0);
      RM_LOG::PRINT(" : ");
      RM_LOG::PRINTLN(STRING(component_.at(it_component->first).name.child.at(index)));
    }
    RM_LOG::PRINTLN(" [Actuator]");
    RM_LOG::PRINT(" -Actuator Name : ");
    RM_LOG::PRINTLN(STRING(component_.at(it_component->first).actuator_name));
    RM_LOG::PRINT(" -ID : ");
    RM_LOG::PRINTLN("", component_.at(it_component->first).joint_constant.id,0);
    RM_LOG::PRINTLN(" -Joint Axis : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).joint_constant.axis);
    RM_LOG::PRINT(" -Coefficient : ");
    RM_LOG::PRINTLN("", component_.at(it_component->first).joint_constant.coefficient);
    RM_LOG::PRINTLN(" -Limit : ");
    RM_LOG::PRINT("    Maximum :", component_.at(it_component->first).joint_constant.limit.maximum);
    RM_LOG::PRINTLN(", Minimum :", component_.at(it_component->first).joint_constant.limit.minimum);

    RM_LOG::PRINTLN(" [Actuator Value]");
    RM_LOG::PRINTLN(" -Value : ", component_.at(it_component->first).joint_value.position);
    RM_LOG::PRINTLN(" -Velocity : ", component_.at(it_component->first).joint_value.velocity);
    RM_LOG::PRINTLN(" -Acceleration : ", component_.at(it_component->first).joint_value.acceleration);
    RM_LOG::PRINTLN(" -Effort : ", component_.at(it_component->first).joint_value.effort);

    RM_LOG::PRINTLN(" [Constant]");
    RM_LOG::PRINTLN(" -Relative Position from parent component : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).relative.pose_from_parent.position);
    RM_LOG::PRINTLN(" -Relative Orientation from parent component : ");
    RM_LOG::PRINT_MATRIX(component_.at(it_component->first).relative.pose_from_parent.orientation);
    RM_LOG::PRINT(" -Mass : ");
    RM_LOG::PRINTLN("", component_.at(it_component->first).relative.inertia.mass);
    RM_LOG::PRINTLN(" -Inertia Tensor : ");
    RM_LOG::PRINT_MATRIX(component_.at(it_component->first).relative.inertia.inertia_tensor);
    RM_LOG::PRINTLN(" -Center of Mass : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).relative.inertia.center_of_mass);

    RM_LOG::PRINTLN(" [Variable]");
    RM_LOG::PRINTLN(" -Position : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).pose_from_world.kinematic.position);
    RM_LOG::PRINTLN(" -Orientation : ");
    RM_LOG::PRINT_MATRIX(component_.at(it_component->first).pose_from_world.kinematic.orientation);
    RM_LOG::PRINTLN(" -Linear Velocity : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).pose_from_world.dynamic.linear.velocity);
    RM_LOG::PRINTLN(" -Linear acceleration : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).pose_from_world.dynamic.linear.acceleration);
    RM_LOG::PRINTLN(" -Angular Velocity : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).pose_from_world.dynamic.angular.velocity);
    RM_LOG::PRINTLN(" -Angular acceleration : ");
    RM_LOG::PRINT_VECTOR(component_.at(it_component->first).pose_from_world.dynamic.angular.acceleration);
  }
  RM_LOG::PRINTLN("---------------------------------------------");
}

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setWorldPose(PoseValue world_pose)
{
  world_.pose = world_pose;
}

void Manipulator::setWorldKinematicPose(KinematicPose world_kinematic_pose)
{
  world_.pose.kinematic = world_kinematic_pose;
}

void Manipulator::setWorldPosition(Eigen::Vector3d world_position)
{
  world_.pose.kinematic.position = world_position;
}

void Manipulator::setWorldOrientation(Eigen::Matrix3d world_orientation)
{
  world_.pose.kinematic.orientation = world_orientation;
}

void Manipulator::setWorldDynamicPose(DynamicPose world_dynamic_pose)
{
  world_.pose.dynamic = world_dynamic_pose;
}

void Manipulator::setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity)
{
  world_.pose.dynamic.linear.velocity = world_linear_velocity;
}

void Manipulator::setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity)
{
  world_.pose.dynamic.angular.velocity = world_angular_velocity;
}

void Manipulator::setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration)
{
  world_.pose.dynamic.linear.acceleration = world_linear_acceleration;
}

void Manipulator::setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration)
{
  world_.pose.dynamic.angular.acceleration = world_angular_acceleration;
}

void Manipulator::setComponent(Name component_name, Component component)
{
  component_.at(component_name) = component;
}

void Manipulator::setComponentActuatorName(Name component_name, Name actuator_name)
{
  component_.at(component_name).actuator_name = actuator_name;
}

void Manipulator::setComponentPoseFromWorld(Name component_name, PoseValue pose_to_world)
{
  if (component_.find(component_name) != component_.end())
  {
    component_.at(component_name).pose_from_world = pose_to_world;
  }
  else
  {
    RM_LOG::ERROR("[setComponentPoseFromWorld] Wrong name.");
  }
}

void Manipulator::setComponentKinematicPoseFromWorld(Name component_name, KinematicPose pose_to_world)
{
  if (component_.find(component_name) != component_.end())
  {
    component_.at(component_name).pose_from_world.kinematic = pose_to_world;
  }
  else
  {
    RM_LOG::ERROR("[setComponentKinematicPoseFromWorld] Wrong name.");
  }
}

void Manipulator::setComponentPositionFromWorld(Name component_name, Eigen::Vector3d position_to_world)
{
  if (component_.find(component_name) != component_.end())
  {
    component_.at(component_name).pose_from_world.kinematic.position = position_to_world;
  }
  else
  {
    RM_LOG::ERROR("[setComponentPositionFromWorld] Wrong name.");
  }
}

void Manipulator::setComponentOrientationFromWorld(Name component_name, Eigen::Matrix3d orientation_to_wolrd)
{
  if (component_.find(component_name) != component_.end())
  {
    component_.at(component_name).pose_from_world.kinematic.orientation = orientation_to_wolrd;
  }
  else
  {
    RM_LOG::ERROR("[setComponentOrientationFromWorld] Wrong name.");
  }
}

void Manipulator::setComponentDynamicPoseFromWorld(Name component_name, DynamicPose dynamic_pose)
{
  if (component_.find(component_name) != component_.end())
  {
    component_.at(component_name).pose_from_world.dynamic = dynamic_pose;
  }
  else
  {
    RM_LOG::ERROR("[setComponentDynamicPoseFromWorld] Wrong name.");
  }
}

void Manipulator::setJointPosition(Name name, double position)
{
  component_.at(name).joint_value.position = position;
}

void Manipulator::setJointVelocity(Name name, double velocity)
{
  component_.at(name).joint_value.velocity = velocity;
}

void Manipulator::setJointAcceleration(Name name, double acceleration)
{
  component_.at(name).joint_value.acceleration = acceleration;
}

void Manipulator::setJointEffort(Name name, double effort)
{
  component_.at(name).joint_value.effort = effort;
}

void Manipulator::setJointValue(Name name, JointValue joint_value)
{
  component_.at(name).joint_value = joint_value;
}

void Manipulator::setAllActiveJointPosition(std::vector<double> joint_position_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == ACTIVE_JOINT_COMPONENT)
    {
      component_.at(it_component->first).joint_value.position = joint_position_vector.at(index);
      index++;
    }
  }
}

void Manipulator::setAllActiveJointValue(std::vector<JointValue> joint_value_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == ACTIVE_JOINT_COMPONENT)
    {
      component_.at(it_component->first).joint_value.position = joint_value_vector.at(index).position;
      component_.at(it_component->first).joint_value.velocity = joint_value_vector.at(index).velocity;
      component_.at(it_component->first).joint_value.acceleration = joint_value_vector.at(index).acceleration;
      component_.at(it_component->first).joint_value.effort = joint_value_vector.at(index).effort;
      index++;
    }
  }
}

void Manipulator::setAllJointPosition(std::vector<double> joint_position_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == ACTIVE_JOINT_COMPONENT || component_.at(it_component->first).component_type == PASSIVE_JOINT_COMPONENT)
    {
      component_.at(it_component->first).joint_value.position = joint_position_vector.at(index);
      index++;
    }
  }
}


void Manipulator::setAllJointValue(std::vector<JointValue> joint_value_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == ACTIVE_JOINT_COMPONENT || component_.at(it_component->first).component_type == PASSIVE_JOINT_COMPONENT)
    {
      component_.at(it_component->first).joint_value = joint_value_vector.at(index);
      index++;
    }
  }
}

void Manipulator::setAllToolPosition(std::vector<double> tool_position_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == TOOL_COMPONENT)
    {
      component_.at(it_component->first).joint_value.position = tool_position_vector.at(index);
      index++;
    }
  }
}

void Manipulator::setAllToolValue(std::vector<JointValue> tool_value_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == TOOL_COMPONENT)
    {
      component_.at(it_component->first).joint_value = tool_value_vector.at(index);
      index++;
    }
  }
}

///////////////////////////////Get function//////////////////////////////////

int8_t Manipulator::getDOF()
{
  return dof_;
}

Name Manipulator::getWorldName()
{
  return world_.name;
}

Name Manipulator::getWorldChildName()
{
  return world_.child;
}

PoseValue Manipulator::getWorldPose()
{
  return world_.pose;
}

KinematicPose Manipulator::getWorldKinematicPose()
{
  return world_.pose.kinematic;
}

Eigen::Vector3d Manipulator::getWorldPosition()
{
  return world_.pose.kinematic.position;
}

Eigen::Matrix3d Manipulator::getWorldOrientation()
{
  return world_.pose.kinematic.orientation;
}

DynamicPose Manipulator::getWorldDynamicPose()
{
  return world_.pose.dynamic;
}

int8_t Manipulator::getComponentSize()
{
  return component_.size();
}

std::map<Name, Component> Manipulator::getAllComponent()
{
  return component_;
}

std::map<Name, Component>::iterator Manipulator::getIteratorBegin()
{
  return component_.begin();
}

std::map<Name, Component>::iterator Manipulator::getIteratorEnd()
{
  return component_.end();;
}

Component Manipulator::getComponent(Name name)
{
  return component_.at(name);
}

Name Manipulator::getComponentActuatorName(Name component_name)
{
  return component_.at(component_name).actuator_name;
}

Name Manipulator::getComponentParentName(Name name)
{
  return component_.at(name).name.parent;
}

std::vector<Name> Manipulator::getComponentChildName(Name name)
{
  return component_.at(name).name.child;
}

PoseValue Manipulator::getComponentPoseFromWorld(Name name)
{
  return component_.at(name).pose_from_world;
}

KinematicPose Manipulator::getComponentKinematicPoseFromWorld(Name name)
{
  return component_.at(name).pose_from_world.kinematic;
}

Eigen::Vector3d Manipulator::getComponentPositionFromWorld(Name name)
{
  return component_.at(name).pose_from_world.kinematic.position;
}

Eigen::Matrix3d Manipulator::getComponentOrientationFromWorld(Name name)
{
  return component_.at(name).pose_from_world.kinematic.orientation;
}

DynamicPose Manipulator::getComponentDynamicPoseFromWorld(Name name)
{
  return component_.at(name).pose_from_world.dynamic;
}

KinematicPose Manipulator::getComponentRelativePoseFromParent(Name name)
{
  return component_.at(name).relative.pose_from_parent;
}

Eigen::Vector3d Manipulator::getComponentRelativePositionFromParent(Name name)
{
  return component_.at(name).relative.pose_from_parent.position;
}

Eigen::Matrix3d Manipulator::getComponentRelativeOrientationFromParent(Name name)
{
  return component_.at(name).relative.pose_from_parent.orientation;
}

int8_t Manipulator::getId(Name name)
{
  return component_.at(name).joint_constant.id;
}

double Manipulator::getCoefficient(Name name)
{
  return component_.at(name).joint_constant.coefficient;
}

Eigen::Vector3d Manipulator::getAxis(Name name)
{
  return component_.at(name).joint_constant.axis;
}

double Manipulator::getJointPosition(Name name)
{
  return component_.at(name).joint_value.position;
}

double Manipulator::getJointVelocity(Name name)
{
  return component_.at(name).joint_value.velocity;
}

double Manipulator::getJointAcceleration(Name name)
{
  return component_.at(name).joint_value.acceleration;
}

double Manipulator::getJointEffort(Name name)
{
  return component_.at(name).joint_value.effort;
}

JointValue Manipulator::getJointValue(Name name)
{
  return component_.at(name).joint_value;
}

double Manipulator::getComponentMass(Name name)
{
  return component_.at(name).relative.inertia.mass;
}

Eigen::Matrix3d Manipulator::getComponentInertiaTensor(Name name)
{
  return component_.at(name).relative.inertia.inertia_tensor;
}

Eigen::Vector3d Manipulator::getComponentCenterOfMass(Name name)
{
  return component_.at(name).relative.inertia.center_of_mass;
}

std::vector<double> Manipulator::getAllJointPosition()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT) || checkComponentType(it_component->first, PASSIVE_JOINT_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value.position);
    }
  }
  return result_vector;
}

std::vector<JointValue> Manipulator::getAllJointValue()
{
  std::vector<JointValue> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT) || checkComponentType(it_component->first, PASSIVE_JOINT_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value);
    }
  }
  return result_vector;
}

std::vector<double> Manipulator::getAllActiveJointPosition()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value.position);
    }
  }
  return result_vector;
}

std::vector<JointValue> Manipulator::getAllActiveJointValue()
{
  std::vector<JointValue> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value);
    }
  }
  return result_vector;
}

std::vector<double> Manipulator::getAllToolPosition()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, TOOL_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value.position);
    }
  }
  return result_vector;
}


std::vector<JointValue> Manipulator::getAllToolValue()
{
  std::vector<JointValue> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, TOOL_COMPONENT))
    {
      result_vector.push_back(component_.at(it_component->first).joint_value);
    }
  }
  return result_vector;
}

std::vector<uint8_t> Manipulator::getAllJointID()
{
  std::vector<uint8_t> joint_id;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT) || checkComponentType(it_component->first, PASSIVE_JOINT_COMPONENT))
    {
      joint_id.push_back(component_.at(it_component->first).joint_constant.id);
    }
  }
  return joint_id;
}

std::vector<uint8_t> Manipulator::getAllActiveJointID()
{
  std::vector<uint8_t> active_joint_id;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT))
    {
      active_joint_id.push_back(component_.at(it_component->first).joint_constant.id);
    }
  }
  return active_joint_id;
}


std::vector<Name> Manipulator::getAllToolComponentName()
{
  std::vector<Name> tool_name;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, TOOL_COMPONENT))
    {
      tool_name.push_back(it_component->first);
    }
  }
  return tool_name;
}

std::vector<Name> Manipulator::getAllActiveJointComponentName()
{
  std::vector<Name> active_joint_name;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (checkComponentType(it_component->first, ACTIVE_JOINT_COMPONENT))
    {
      active_joint_name.push_back(it_component->first);
    }
  }
  return active_joint_name;
}



bool Manipulator::checkLimit(Name component_name, double value)
{
  if(component_.at(component_name).joint_constant.limit.maximum < value)
    return false;
  else if(component_.at(component_name).joint_constant.limit.minimum > value)
    return false;
  else
    return true;
}

bool Manipulator::checkComponentType(Name component_name, ComponentType component_type)
{
  if(component_.at(component_name).component_type == component_type)
    return true;
  else
    return false;
}

Name Manipulator::findComponentNameFromId(int8_t id)
{
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).joint_constant.id == id)
    {
      return it_component->first;
    }
  }
  return {};
}
////////////////////////////////////////////////////////////
