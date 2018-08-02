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

#include "../../../include/open_manipulator/OMManager.h"

using namespace std, Eigen;

///////////////////////////*initialize function*/////////////////////////////
void Manipulator::addWorld(Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error = false)
{
  world_.name_ = world_name;
  world_.child_ = child_name;
  world_.pose_.position = world_position;
  world_.pose_.orientation = world_orientation;
  world_.origin_.velocity = 0.0;
  world_.origin_.acceleration = 0.0;
}

void Manipulator::addComponent(Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t joint_actuator_id = -1, 
                  Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error = false)
{
  Component temp_component;
  temp_component.parent_ = parent_name;
  temp_component.child_.push_back(child_name);
  temp_component.relative_to_parent_.position = relative_position;
  temp_component.relative_to_parent_.orientation = relative_orientation;
  temp_component.pose_to_world_.position = Vector3f::Zero();
  temp_component.pose_to_world_.orientation = Matrix3f::Identity(3,3);
  temp_component.origin_.velocity = VectorXf::Zero();
  temp_component.origin_.acceleration = VectorXf::Zero();
  temp_component.joint_.id = joint_actuator_id;
  temp_component.joint_.axis = axis_of_rotation;
  temp_component.joint_.angle = 0.0;
  temp_component.joint_.velocity = 0.0;
  temp_component.joint_.acceleration = 0.0;
  temp_component.tool_.id = -1;
  temp_component.tool_.on_off = false;
  temp_component.tool_.value = 0.0;
  temp_component.inertia_.mass = mass;
  temp_component.inertia_.inertia_tensor = inertia_tensor;
  temp_component.inertia_.center_of_mass = center_of_mass;
  component_.insert(make_pair(me_name,temp_component));
}

void Manipulator::addComponentChild(Name me_name, NAME child_name, bool *error = false)
{
  component_.at(me_name).child_.push_back(child_name);
}

void Manipulator::addTool(Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1,
              float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero())
{
  Component temp_component;
  temp_component.parent_ = parent_name;
  temp_component.relative_to_parent_.position = relative_position;
  temp_component.relative_to_parent_.orientation = relative_orientation;
  temp_component.pose_to_world_.position = Vector3f::Zero();
  temp_component.pose_to_world_.orientation = Matrix3f::Identity(3,3);
  temp_component.origin_.velocity = VectorXf::Zero();
  temp_component.origin_.acceleration = VectorXf::Zero();
  temp_component.joint_.id = -1;
  temp_component.joint_.axis = Vector3f::Zero();
  temp_component.joint_.angle = 0.0;
  temp_component.joint_.velocity = 0.0;
  temp_component.joint_.acceleration = 0.0;
  temp_component.tool_.id = tool_id;
  temp_component.tool_.on_off = false;
  temp_component.tool_.value = 0.0;
  temp_component.inertia_.mass = mass;
  temp_component.inertia_.inertia_tensor = inertia_tensor;
  temp_component.inertia_.center_of_mass = center_of_mass;
  component_.insert(make_pair(me_name,temp_component));
}

void Manipulator::checkManipulatorSetting(bool *error = false)
{
  
}
/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setWorldPose(Pose world_pose, bool *error = false)
{
  world_.pose_ = world_pose;
}

void Manipulator::setWorldPosition(Vector3f world_position, bool *error = false)
{
  world_.pose_.position = world_position;
}

void Manipulator::setWorldOrientation(Matrix3f world_orientation, bool *error = false)
{
  world_.pose_.orientation = world_orientation;
}

void Manipulator::setWorldState(State world_state, bool *error = false)
{
  world_.origin_ = world_state;
}

void Manipulator::setWorldVelocity(VectorXf world_velocity, bool *error = false)
{
  world_.origin_.velocity = world_velocity;
}

void Manipulator::setWorldAcceleration(VectorXf world_acceleration, bool *error = false)
{
  world_.origin_.acceleration = world_acceleration;
}

void Manipulator::setComponent(Name name, Component component, bool *error = false)
{
  if(component_.find(name) != component_.end())
  {
    component_.insert(name, component);
  }
  else
  {
    //error
  } 
}

void Manipulator::setComponentPoseToWorld(Name name Pose pose_to_world, bool *error = false)
{
  if(component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world_ = pose_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world, bool *error = false)
{
  if(component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world_.position = position_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd, bool *error = false)
{
  if(component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world_.orientation = orientation_to_wolrd;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentStateToWorld(Name name State state_to_world, bool *error = false)
{
  if(component_.find(name) != component_.end())
  {
    component_.at(name).origin_ = state_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentVelocityToWorld(Name name, VectorXf velocity, bool *error = false)
{
  if(velocity.size()!=6)
  {
    //error
  }
  else
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).origin_.velocity = velocity;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentAccelerationToWorld(Name name, VectorXf accelaration, bool *error = false)
{
  if(accelaration.size()!=6)
  {
    //error
  }
  else
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).origin_.accelaration = accelaration;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointAngle(Name name, float angle, bool *error = false)
{
  if(component_.at(name).tool_.id_ > 0)
  {
    //error
  }
  else
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).joint_.angle = angle;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointVelocity(Name name, float angular_velocity, bool *error = false)
{
  if(component_.at(name).tool_.id_ > 0)
  {
    //error
  }
  else
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).joint_.velocity = angular_velocity;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentJointAcceleration(Name name, float angular_acceleration, bool *error = false)
{
  if(component_.at(name).tool_.id_ > 0)
  {
    //error
  }
  else
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).joint_.acceleration = angular_acceleration;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setComponentToolOnOff(Name name, bool on_off, bool *error = false)
{
  if(component_.at(name).tool_.id_ > 0)
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).tool_.on_off = on_off;
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

void Manipulator::setComponentToolValue(Name name, float value, bool *error = false)
{
  if(component_.at(name).tool_.id_ > 0)
  {
    if(component_.find(name) != component_.end())
    {
      component_.at(name).tool_.value = value;
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

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Get function//////////////////////////////////
int8_t Manipulator::getDOF(bool *error = false)
{
  return dof_;
}

int8_t Manipulator::getComponentSize(bool *error = false)
{
  return component_.size();
}

Name Manipulator::getWorldName(bool *error = false)
{
  return world_.name_;
}

Name Manipulator::getWorldChildName(bool *error = false)
{
  return world_.child_;
}

Pose Manipulator::getWorldPose(bool *error = false)
{
  return world_.pose_;
}

Vector3f Manipulator::getWorldPosition(bool *error = false)
{
  return world_.pose_.position;
}

Matrix3f Manipulator::getWorldOrientation(bool *error = false)
{
  return world_.pose_.orientation;
}

State Manipulator::getWorldState(bool *error = false)
{
  return world_.origin_;
}

VectorXf Manipulator::getWorldVelocity(bool *error = false)
{
  return world_.origin_.velocity;
}

VectorXf Manipulator::getWorldAcceleration(bool *error = false)
{
  return world_.origin_.acceleration;
}

map<Name, Component> Manipulator::getAllComponent(bool *error = false)
{
  return component_;
}

Component Manipulator::getComponent(Name name, bool *error = false)
{
  return component_.at(name);
}

Name Manipulator::getComponentParentName(Name name, bool *error = false)
{
  return component_.at(name).parent_;
}

vector<NAME> Manipulator::getComponentChildName(Name name, bool *error = false)
{
  return component_.at(name).child_;
}

Pose Manipulator::getComponentPoseToWorld(Name name, bool *error = false)
{
  return component_.at(name).pose_to_world_;
}

Vector3f Manipulator::getComponentPositionToWorld(Name name, bool *error = false)
{
  return component_.at(name).pose_to_world_.position;
}

Matrix3f Manipulator::getComponentOrientationToWorld(Name name, bool *error = false)
{
  return component_.at(name).pose_to_world_.orientation;
}

State Manipulator::getComponentStateToWorld(Name name, bool *error = false)
{
  return component_.at(name).origin_;
}

VectorXf Manipulator::getComponentVelocityToWorld(Name name, bool *error = false)
{
  return component_.at(name).origin_.velocity;
}

VectorXf Manipulator::getComponentAccelerationToWorld(Name name, bool *error = false)
{
  return component_.at(name).origin_.acceleration;
}

Pose Manipulator::getComponentRelativePoseToParent(Name name, bool *error = false)
{
  return component_.at(name).relative_to_parent_;
}

Vector3f Manipulator::getComponentRelativePositionToParent(Name name, bool *error = false)
{
  return component_.at(name).relative_to_parent_.position;
}

Matrix3f Manipulator::getComponentRelativeOrientationToParent(Name name, bool *error = false)
{
  return component_.at(name).relative_to_parent_.orientation;
}

Joint Manipulator::getComponentJoint(Name name, bool *error = false)
{
  return component_.at(name).joint_;
}

int8_t Manipulator::getComponentJointId(Name name, bool *error = false)
{
  return component_.at(name).joint_.id;
}

Vector3f Manipulator::getComponentJointAxis(Name name, bool *error = false)
{
  return component_.at(name).joint_.axis;
} 
 
float Manipulator::getComponentJointAngle(Name name, bool *error = false)
{
  return component_.at(name).joint_.angle;
}

float Manipulator::getComponentJointVelocity(Name name, bool *error = false)
{
  return component_.at(name).joint_.velocity;
}

float Manipulator::getComponentJointAcceleration(Name name, bool *error = false)
{
  return component_.at(name).joint_.acceleration;
}

Tool Manipulator::getComponentTool(Name name, bool *error = false)
{
  return component_.at(name).tool_;
}

int8_t Manipulator::getComponentToolId(Name name, bool *error = false)
{
  return component_.at(name).tool_.id;
}

bool Manipulator::getComponentToolOnOff(Name name, bool *error = false)
{
  return component_.at(name).tool_.on_off;
}

float Manipulator::getComponentToolValue(Name name, bool *error = false)
{
  return component_.at(name).tool_.value;
}

float Manipulator::getComponentMass(Name name, bool *error = false)
{
  return component_.at(name).inertia_.mass;
}

Matrix3f Manipulator::getComponentInertiaTensor(Name name, bool *error = false)
{
  return component_.at(name).inertia_.inertia_tensor;
}

Vector3f Manipulator::getComponentCenterOfMass(Name name, bool *error = false)
{
  return component_.at(name).inertia_.center_of_mass;
}
