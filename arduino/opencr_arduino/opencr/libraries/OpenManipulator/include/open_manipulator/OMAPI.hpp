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
   
#ifndef OMAPI_HPP_
#define OMAPI_HPP_

#include <RTOS.h>
#include <Eigen.h>
#include <OpenManipulator.h>

osMutexDef(om_mutex);
osMutexId(om_mutex_id);

namespace MUTEX
{
void create(){ om_mutex_id = osMutexCreate(osMutex(om_mutex)); }
void wait(){ osMutexWait(om_mutex_id, osWaitForever); }
void release(){ osMutexRelease(om_mutex_id); }
} // namespace THREAD

namespace OPEN_MANIPULATOR
{
// Connect functions
namespace ACTUATOR
{
bool (*setAllJointAngle)(float*) = NULL;
bool (*setJointAngle)(uint8_t, float) = NULL;
float* (*getAngle)() = NULL;
} // namespace ACTUATOR

void connectSetAllJointAngle(bool (*fp)(float*)){ ACTUATOR::setAllJointAngle = fp; }
void connectSetJointAngle(bool (*fp)(uint8_t, float)){ ACTUATOR::setJointAngle = fp; }
void connectSetGetAngle(float* (*fp)()){ ACTUATOR::getAngle = fp; }

namespace KINEMATICS
{
void (*foward)(void) = NULL;
void (*inverse)(void) = NULL;
} // namespace KINEMATICS

void connectForward(void (*fp)(void)){KINEMATICS::foward = fp;}
void connectInverse(void (*fp)(void)){KINEMATICS::inverse = fp;}

namespace PATH
{
void (*line)(void) = NULL;
void (*arc)(void) = NULL;
void (*custom)(void) = NULL;
} // namespace PATH

void connectLine(void (*fp)(void)){PATH::line = fp;}
void connectArc(void (*fp)(void)){PATH::arc = fp;}
void connectCustom(void (*fp)(void)){PATH::custom = fp;}

// namespace Manipulator
namespace Manipulator
{
  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Manipulator *manipulator, Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error)
  {
    manipulator->addWorld(world_name, child_name, world_position, world_orientation, error);
  }

  void addComponent(Manipulator *manipulator, Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error)
  {
    manipulator->addComponent(me_name, parent_name, child_name, relative_position, relative_orientation, actuator_id, axis_of_rotation, mass, inertia_tensor, center_of_mass, *error);
  }

  void addTool(Manipulator *manipulator, Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1,
              float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero())
  {
    manipulator->addTool(me_name, parent_name, relative_position, relative_orientation, tool_id, mass, inertia_tensor, Vector3f center_of_mass);
  }

  void addComponentChild(Manipulator *manipulator, Name me_name, NAME child_name, bool *error)
  {
    manipulator->addComponentChild(me_name, child_name, error);
  }

  void checkManipulatorSetting(Manipulator *manipulator, bool *error)
  {
    manipulator->checkManipulatorSetting(error);
  }

  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Manipulator *manipulator, Pose world_pose, bool *error)
  {
    manipulator->setWorldPose();
  }

  void setWorldPosition(Manipulator *manipulator, Vector3f world_position, bool *error)
  {
    manipulator->setWorldPosition(world_position, error);
  }

  void setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation, bool *error)
  {
    manipulator->setWorldOrientation(world_orientation, error);
  }

  void setWorldState(Manipulator *manipulator, State world_state, bool *error)
  {
    manipulator->setWorldState(world_state, error);
  }

  void setWorldVelocity(Manipulator *manipulator, Vector6f world_velocity, bool *error)
  {
    manipulator->setWorldVelocity(world_velocity, error);
  }

  void setWorldAcceleration(Manipulator *manipulator, Vector6f world_acceleration, bool *error)
  {
    manipulator->setWorldAcceleration(world_acceleration, error);
  }
  
  void setComponent(Manipulator *manipulator, Name name, Component component, bool *error)
  {
    manipulator->setComponent(name, component, error);
  }

  void setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world, bool *error)
  {
    manipulator->setComponentPoseToWorld(name, pose_to_world, error);
  }

  void setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world, bool *error)
  {
    manipulator->setComponentPositionToWorld(name, position_to_world, error);
  }

  void setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd, bool *error)
  {
    manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd, error);
  }

  void setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world, bool *error)
  {
    manipulator->setComponentStateToWorld(name, state_to_world, error);
  }

  void setComponentVelocityToWorld(Manipulator *manipulator, Name name, Vector6f velocity, bool *error)
  {
    manipulator->setComponentVelocityToWorld(name, velocity, error);
  }

  void setComponentAccelerationToWorld(Manipulator *manipulator, Name name, Vector6f accelaration, bool *error)
  {
    manipulator->setComponentAccelerationToWorld(name, accelaration, error);
  }

  void setComponentJointAngle(Manipulator *manipulator, Name name, float angle, bool *error)
  {
    manipulator->setComponentJointAngle(name, angle, error);
  }

  void setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity, bool *error)
  {
    manipulator->setComponentJointVelocity(name, angular_velocity, error);
  }

  void setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration, bool *error)
  {
    manipulator->setComponentJointAcceleration(name, angular_acceleration, error);
  }

  void setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off, bool *error)
  {
    manipulator->setComponentToolOnOff(name, on_off, error);
  }

  void setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value, bool *error)
  {
    manipulator->setComponentToolValue(name, actuator_value, error);
  }

  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Get function//////////////////////////////////
  int getDOF(Manipulator *manipulator, bool *error)
  {
    return manipulator->getDOF(error);
  }

  int8_t getComponentSize(Manipulator *manipulator, bool *error)
  {
    return manipulator->getComponentSize(error);
  }

  Name getWorldName((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldName(error);
  }

  Name getWorldChildName((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldChildName(error);
  }

  Pose getWorldPose((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldPose(error);
  }

  Vector3f getWorldPosition((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldPosition(error);
  }

  Matrix3f getWorldOrientation((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldOrientation(error);
  }

  State getWorldState((Manipulator *manipulator, bool *error)(error);
  {
    return manipulator->getWorldState(error);
  }

  Vector6f getWorldVelocity((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldVelocity(error);
  }

  Vector6f getWorldAcceleration((Manipulator *manipulator, bool *error)
  {
    return manipulator->getWorldAcceleration(error);
  }

  map<Name, Component> getAllComponent((Manipulator *manipulator, bool *error)
  {
    return manipulator->getAllComponent(error);
  }

  Component getComponent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponent(name, error);
  }

  Name getComponentParentName(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentParentName(name, error);
  }

  vector<NAME> getComponentChildName(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentChildName(name, error);
  }

  Pose getComponentPoseToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentPoseToWorld(name, error);
  }

  Vector3f getComponentPositionToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentPositionToWorld(name, error);
  }

  Matrix3f getComponentOrientationToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentOrientationToWorld(name, error);
  }

  State getComponentStateToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentStateToWorld(name, error);
  }

  Vector6f getComponentVelocityToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentVelocityToWorld(name, error);
  }

  Vector6f getComponentAccelerationToWorld(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentAccelerationToWorld(name, error);
  }

  Pose getComponentRelativePoseToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativePoseToParent(name, error);
  }

  Vector3f getComponentRelativePositionToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativePositionToParent(name, error);
  }

  Matrix3f getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentRelativeOrientationToParent(name, error);
  }

  Joint getComponentJoint(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJoint(name, error);
  }

  int8_t getComponentJointId(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointId(name, error);
  }

  Vector3f getComponentJointAxis(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAxis(name, error);
  }

  float getComponentJointAngle(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAngle(name, error);
  }

  float getComponentJointVelocity(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointVelocity(name, error);
  }

  float getComponentJointAcceleration(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentJointAcceleration(name, error);
  }

  Tool getComponentTool(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentTool(name, error);
  }

  int8_t getComponentToolId(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolId(name, error);
  }

  bool getComponentToolOnOff(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolOnOff(name, error);
  }

  float getComponentToolValue(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentToolValue(name, error);
  }

  float getComponentMass(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentMass(name, error);
  }

  Matrix3f getComponentInertiaTensor(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentInertiaTensor(name, error);
  }

  Pose getComponentCenterOfMassPose(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassPose(name, error);
  }

  Vector3f getComponentCenterOfMassPosition(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassPosition(name, error);
  }

  Matrix3f getComponentCenterOfMassOrientation(Manipulator *manipulator, Name name, bool *error)
  {
    return manipulator->getComponentCenterOfMassOrientation(name, error);
  }
  /////////////////////////////////////////////////////////////////////////////
}





// Thread
void Thread_Robot_State(void const *argument)
{
  (void) argument;

  LOG::init();
  MUTEX::create();

  for(;;)
  {
    KINEMATICS::inverse();

  // MUTEX::wait();
  //   float* angle_ptr = ACTUATOR::getAngle();
  // MUTEX::release();
  //   LOG::INFO("angle : " + String(angle_ptr[0])); 

  osDelay(10);
  }
}

} // namespace OPEN_MANIPULATOR

#endif // OMAPI_HPP_  
   
   
   
   
   
   
  