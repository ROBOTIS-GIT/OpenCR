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
#include <Eigen.h>
#include <map>
#include <vector>

#include "OMDebug.h"

using namespace Eigen;

typedef int8_t Name;

typedef struct
{
  Vector3f position;
  Matrix3f orientation;
} Pose;

typedef struct
{
  VectorXf velocity;
  VectorXf acceleration;
} State;

typedef struct
{
  int8_t id;
  Vector3f axis;
  float coefficient; //actuator angle to joint angle
  float angle;
  float velocity;
  float acceleration;
} Joint;

typedef struct
{
  int8_t id;
  bool on_off;
  float coefficient; //actuator value to tool value
  float value;       //m or rad
} Tool;

typedef struct
{
  float mass;
  Matrix3f inertia_tensor;
  Vector3f center_of_mass;
} Inertia;

typedef struct
{
  Name name;
  Name child;
  Pose pose;
  State origin;
} World;

typedef struct
{
  Name parent;
  std::vector<Name> child;
  Pose relative_to_parent;
  Pose pose_to_world;
  State origin;
  Joint joint;
  Tool tool;
  Inertia inertia;
} Component;

namespace OM_MANAGER
{
class Manipulator
{
private:
  int8_t dof_;
  World world_;
  std::map<Name, Component> component_;
  std::map<Name, Component>::iterator it_component_;

  /////////////////////////////Parameter list///////////////////////////////
  /*
  dof_
  world_.name
  world_.child
  world_.pose.position
  world_.pose.orientation
  world_.origin.velocity
  world_.origin.acceleration
  component_.at(name).parent
  component_.at(name).child.at(i)
  component_.at(name).relative_to_parent.position
  component_.at(name).relative_to_parent.orientation
  component_.at(name).pose_to_world.position
  component_.at(name).pose_to_world.orientation
  component_.at(name).origin.velocity
  component_.at(name).origin.acceleration
  component_.at(name).joint.id
  component_.at(name).joint.coefficient
  component_.at(name).joint.axis
  component_.at(name).joint.angle
  component_.at(name).joint.velocity
  component_.at(name).joint.acceleration
  component_.at(name).tool.id
  component_.at(name).tool.coefficient
  component_.at(name).tool.on_off
  component_.at(name).tool.value
  component_.at(name).inertia.mass
  component_.at(name).inertia.inertia_tensor
  component_.at(name).inertia.center_of_mass
  */
  /////////////////////////////////////////////////////////////////////////////

public:
  Manipulator() : dof_(0){};
  virtual ~Manipulator(){};

  ///////////////////////////initialize function/////////////////////////////

  void addWorld(Name world_name,
                Name child_name,
                Vector3f world_position = Vector3f::Zero(),
                Matrix3f world_orientation = Matrix3f::Identity(3, 3));

  void addComponent(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Vector3f relative_position,
                    Matrix3f relative_orientation,
                    Vector3f axis_of_rotation = Vector3f::Zero(),
                    int8_t joint_actuator_id = -1,
                    float coefficient = 1,
                    float mass = 0.0,
                    Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
                    Vector3f center_of_mass = Vector3f::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Vector3f relative_position,
               Matrix3f relative_orientation,
               int8_t tool_id = -1,
               float coefficient = 1,
               float mass = 0.0,
               Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
               Vector3f center_of_mass = Vector3f::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  ///////////////////////////////Set function//////////////////////////////////

  void setWorldPose(Pose world_pose);
  void setWorldPosition(Vector3f world_position);
  void setWorldOrientation(Matrix3f world_orientation);
  void setWorldState(State world_state);
  void setWorldVelocity(VectorXf world_velocity);
  void setWorldAcceleration(VectorXf world_acceleration);

  void setComponent(Name name, Component component, bool *error = NULL);
  void setComponentPoseToWorld(Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name name, Vector3f position_to_world);
  void setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd);
  void setComponentStateToWorld(Name name, State state_to_world);
  void setComponentVelocityToWorld(Name name, VectorXf velocity);
  void setComponentAccelerationToWorld(Name name, VectorXf accelaration);
  void setComponentJointAngle(Name name, float angle);
  void setComponentJointVelocity(Name name, float angular_velocity);
  void setComponentJointAcceleration(Name name, float angular_acceleration);
  void setComponentToolOnOff(Name name, bool on_off);
  void setComponentToolValue(Name name, float value);

  void setAllActiveJointAngle(std::vector<float> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////

  int8_t getDOF();

  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Vector3f getWorldPosition();
  Matrix3f getWorldOrientation();
  State getWorldState();
  VectorXf getWorldVelocity();
  VectorXf getWorldAcceleration();

  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseToWorld(Name name);
  Vector3f getComponentPositionToWorld(Name name);
  Matrix3f getComponentOrientationToWorld(Name name);
  State getComponentStateToWorld(Name name);
  VectorXf getComponentVelocityToWorld(Name name);
  VectorXf getComponentAccelerationToWorld(Name name);
  Pose getComponentRelativePoseToParent(Name name);
  Vector3f getComponentRelativePositionToParent(Name name);
  Matrix3f getComponentRelativeOrientationToParent(Name name);
  Joint getComponentJoint(Name name);
  int8_t getComponentJointId(Name name);
  float getComponentJointCoefficient(Name name);
  Vector3f getComponentJointAxis(Name name);
  float getComponentJointAngle(Name name);
  float getComponentJointVelocity(Name name);
  float getComponentJointAcceleration(Name name);
  Tool getComponentTool(Name name);
  int8_t getComponentToolId(Name name);
  float getComponentToolCoefficient(Name name);
  bool getComponentToolOnOff(Name name);
  float getComponentToolValue(Name name);
  float getComponentMass(Name name);
  Matrix3f getComponentInertiaTensor(Name name);
  Vector3f getComponentCenterOfMass(Name name);

  std::vector<float> getAllJointAngle();
  std::vector<float> getAllActiveJointAngle();
  std::vector<uint8_t> getAllActiveJointID();
};
} // namespace OM_MANAGER

#endif // OMMANAGER_HPP_