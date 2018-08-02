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

#ifndef OMMANAGER_H_
#define OMMANAGER_H_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>
#include <map>     
#include <vector>     

#include "OMDebug.h"

using namespace std, Eigen;

typedef int8_t NAME

typedef struct
{
  Vector3f position;
  Matrix3f orientation;
} Pose;

typedef struct
{
  VectorXf velocity(6);
  VectorXf acceleration(6);
} State;

typedef struct
{
  int8_t id;
  Vector3f axis;
  float angle;
  float velocity;
  float acceleration;
} Joint;

typedef struct
{
  int8_t id;
  bool on_off;
  float value;   //mm or rad
} Tool;

typedef struct
{
  float mass;
  Matrix3f inertia_tensor;
  Vector3f center_of_mass;
} Inertia;

typedef struct
{
  NAME name_;           
  NAME child_;
  Pose pose_;
  State origin_;
} World;

typedef struct
{          
  NAME parent_;
  vector<NAME> child_;
  Pose relative_to_parent_;
  Pose pose_to_world_;
  State origin_;
  Joint joint_;
  Tool tool_;
  Inertia inertia_;  
} Component;

class Manipulator
{
 private: 
  int8_t dof_;
  World world_;
  map<Name, Component> component_;

 public:
  Manipulator(int8_t dof)
  {
    dof_ = dof;
  }
  virtual ~Manipulator(){};
  //////////////////////////////*Parameter list*///////////////////////////////
  /*
  dof_
  world_.name_
  world_.child_
  world_.pose_.position
  world_.pose_.orientation
  world_.origin_.velocity
  world_.origin_.acceleration
  component_.at(name).parent_
  component_.at(name).child_.at(i)
  component_.at(name).relative_to_parent_.position
  component_.at(name).relative_to_parent_.orientation
  component_.at(name).pose_to_world_.position
  component_.at(name).pose_to_world_.orientation
  component_.at(name).origin_.velocity
  component_.at(name).origin_.acceleration
  component_.at(name).joint_.id
  component_.at(name).joint_.axis
  component_.at(name).joint_.angle
  component_.at(name).joint_.velocity
  component_.at(name).joint_.acceleration
  component_.at(name).tool_.id_
  component_.at(name).tool_.on_off
  component_.at(name).tool_.value
  component_.at(name).inertia_.mass
  component_.at(name).inertia_.inertia_tensor
  component_.at(name).inertia_.center_of_mass
  */
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error = false)
  void addComponent(Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t joint_actuator_id = -1, 
                    Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error = false);
  void addTool(Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1,
               float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero());
  void addComponentChild(Name me_name, NAME child_name, bool *error = false);
  void checkManipulatorSetting(bool *error = false);
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Pose world_pose, bool *error = false);
  void setWorldPosition(Vector3f world_position, bool *error = false);
  void setWorldOrientation(Matrix3f world_orientation, bool *error = false);
  void setWorldState(State world_state, bool *error = false);
  void setWorldVelocity(VectorXf world_velocity, bool *error = false);
  void setWorldAcceleration(VectorXf world_acceleration, bool *error = false);
  void setComponent(Name name, Component component, bool *error = false);
  void setComponentPoseToWorld(Name name Pose pose_to_world, bool *error = false);
  void setComponentPositionToWorld(Name name, Vector3f position_to_world, bool *error = false);
  void setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd, bool *error = false);
  void setComponentStateToWorld(Name name State state_to_world, bool *error = false);
  void setComponentVelocityToWorld(Name name, VectorXf velocity, bool *error = false);
  void setComponentAccelerationToWorld(Name name, VectorXf accelaration, bool *error = false);
  void setComponentJointAngle(Name name, float angle, bool *error = false);
  void setComponentJointVelocity(Name name, float angular_velocity, bool *error = false);
  void setComponentJointAcceleration(Name name,   /////////////////////////////////////////////////////////////////////////////float angular_acceleration, bool *error = false);
  void setComponentToolOnOff(Name name, bool on_off, bool *error = false);
  void setComponentToolValue(Name name, float value, bool *error = false);
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF(bool *error = false);
  Name getWorldName(bool *error = false);
  Name getWorldChildName(bool *error = false);
  Pose getWorldPose(bool *error = false);
  Vector3f getWorldPosition(bool *error = false);
  Matrix3f getWorldOrientation(bool *error = false);
  State getWorldState(bool *error = false);
  VectorXf getWorldVelocity(bool *error = false);
  VectorXf getWorldAcceleration(bool *error = false);
  int8_t getComponentSize(bool *error = false);
  map<Name, Component> getAllComponent(bool *error = false);
  Component getComponent(Name name, bool *error = false);
  Name getComponentParentName(Name name, bool *error = false);
  vector<NAME> getComponentChildName(Name name, bool *error = false);
  Pose getComponentPoseToWorld(Name name, bool *error = false);
  Vector3f getComponentPositionToWorld(Name name, bool *error = false);
  Matrix3f getComponentOrientationToWorld(Name name, bool *error = false);
  State getComponentStateToWorld(Name name, bool *error = false);
  VectorXf getComponentVelocityToWorld(Name name, bool *error = false);
  VectorXf getComponentAccelerationToWorld(Name name, bool *error = false);
  Pose getComponentRelativePoseToParent(Name name, bool *error = false);
  Vector3f getComponentRelativePositionToParent(Name name, bool *error = false);
  Matrix3f getComponentRelativeOrientationToParent(Name name, bool *error = false);
  Joint getComponentJoint(Name name, bool *error = false);
  int8_t getComponentJointId(Name name, bool *error = false);
  Vector3f getComponentJointAxis(Name name, bool *error = false);  
  float getComponentJointAngle(Name name, bool *error = false);
  float getComponentJointVelocity(Name name, bool *error = false);
  float getComponentJointAcceleration(Name name, bool *error = false);
  Tool getComponentTool(Name name, bool *error = false);
  int8_t getComponentToolId(Name name, bool *error = false);
  bool getComponentToolOnOff(Name name, bool *error = false);
  float getComponentToolValue(Name name, bool *error = false);
  float getComponentMass(Name name, bool *error = false);
  Matrix3f getComponentInertiaTensor(Name name, bool *error = false);
  Vector3f getComponentCenterOfMass(Name name, bool *error = false);
  /////////////////////////////////////////////////////////////////////////////
};

#endif // OMMANAGER_HPP_