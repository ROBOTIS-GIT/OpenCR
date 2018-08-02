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
   
#ifndef OMAPI_H_
#define OMAPI_H_

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
void connectGetAngle(float* (*fp)()){ ACTUATOR::getAngle = fp; }

namespace KINEMATICS
{
void (*foward)(Manipulator*, bool*) = NULL;
void (*inverse)(Manipulator*, Name, Pose, bool*) = NULL;
void (*getPassiveJointAngle)(Manipulator*, bool*) = NULL;
} // namespace KINEMATICS

void connectForward(void (*fp)(Manipulator*, bool*)){KINEMATICS::foward = fp;}
void connectInverse(void (*fp)(Manipulator*, Name, Pose, bool*)){KINEMATICS::inverse = fp;}
void connectGetPassiveJointAngle(void (*fp)(Manipulator*, bool*)){KINEMATICS::getPassiveJointAngle = fp;}

namespace PATH
{
void (*line)(void) = NULL;
void (*arc)(void) = NULL;
void (*custom)(void) = NULL;
} // namespace PATH

void connectLine(void (*fp)(void)){PATH::line = fp;}
void connectArc(void (*fp)(void)){PATH::arc = fp;}
void connectCustom(void (*fp)(void)){PATH::custom = fp;}

namespace MATH
{
  float sign(float number);
  Eigen::Vector3f makeEigenVector3(float v1, float v2, float v3);
  Eigen::Matrix3f makeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);
  Eigen::Vector3f matrixLogarithm(Eigen::Matrix3f rotation_matrix);
  Eigen::Matrix3f skewSymmetricMatrix(Eigen::Vector3f v);
  Eigen::Matrix3f rodriguesRotationMatrix(Eigen::Vector3f axis, float angle);  
  Eigen::Matrix3f makeRotationMatrix(float rool, float pitch, float yaw);
  Eigen::Matrix3f makeRotationMatrix(Eigen::Vector3f rotation_vector);
  Eigen::Vector3f makeRotationVector(Eigen::Matrix3f rotation_matrix);
  Eigen::Vector3f differentialPosition(Eigen::Vector3f desired_position, Eigen::Vector3f present_position);
  Eigen::Vector3f differentialOrientation(Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation);
  Eigen::VectorXf differentialPose(Eigen::Vector3f desired_position, Eigen::Vector3f present_position, Eigen::Matrix3f desired_orientation, Eigen::Matrix3f present_orientation);
} // namespace Math

// namespace Manipulator
namespace MANAGER
{
  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Manipulator *manipulator, Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error = false);
  void addComponent(Manipulator *manipulator, Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error = false);
  void addTool(Manipulator *manipulator, Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1, float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error = false);
  void addComponentChild(Manipulator *manipulator, Name me_name, NAME child_name, bool *error = false);
  void checkManipulatorSetting(Manipulator *manipulator, bool *error = false);

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Manipulator *manipulator, Pose world_pose, bool *error = false);
  void setWorldPosition(Manipulator *manipulator, Vector3f world_position, bool *error = false);
  void setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation, bool *error = false);
  void setWorldState(Manipulator *manipulator, State world_state, bool *error = false);
  void setWorldVelocity(Manipulator *manipulator, VectorXf world_velocity, bool *error = false);
  void setWorldAcceleration(Manipulator *manipulator, VectorXf world_acceleration, bool *error = false);
  void setComponent(Manipulator *manipulator, Name name, Component component, bool *error = false);
  void setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world, bool *error = false);
  void setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world, bool *error = false);
  void setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd, bool *error = false);
  void setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world, bool *error = false);
  void setComponentVelocityToWorld(Manipulator *manipulator, Name name, VectorXf velocity, bool *error = false);
  void setComponentAccelerationToWorld(Manipulator *manipulator, Name name, VectorXf accelaration, bool *error = false);
  void setComponentJointAngle(Manipulator *manipulator, Name name, float angle, bool *error = false);
  void setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity, bool *error = false);
  void setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration, bool *error = false);
  void setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off, bool *error = false);
  void setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value, bool *error = false);

  ///////////////////////////////Get function//////////////////////////////////
  int getDOF(Manipulator *manipulator, bool *error = false);
  int8_t getComponentSize(Manipulator *manipulator, bool *error = false);
  Name getWorldName((Manipulator *manipulator, bool *error = false);
  Name getWorldChildName((Manipulator *manipulator, bool *error = false);
  Pose getWorldPose((Manipulator *manipulator, bool *error = false);
  Vector3f getWorldPosition((Manipulator *manipulator, bool *error = false);
  Matrix3f getWorldOrientation((Manipulator *manipulator, bool *error = false);
  State getWorldState((Manipulator *manipulator, bool *error = false);
  VectorXf getWorldVelocity((Manipulator *manipulator, bool *error = false);
  VectorXf getWorldAcceleration((Manipulator *manipulator, bool *error = false);
  map<Name, Component> getAllComponent((Manipulator *manipulator, bool *error = false);
  Component getComponent(Manipulator *manipulator, Name name, bool *error = false);
  Name getComponentParentName(Manipulator *manipulator, Name name, bool *error = false);
  vector<NAME> getComponentChildName(Manipulator *manipulator, Name name, bool *error = false);
  Pose getComponentPoseToWorld(Manipulator *manipulator, Name name, bool *error = false);
  Vector3f getComponentPositionToWorld(Manipulator *manipulator, Name name, bool *error = false);
  Matrix3f getComponentOrientationToWorld(Manipulator *manipulator, Name name, bool *error = false);
  State getComponentStateToWorld(Manipulator *manipulator, Name name, bool *error = false);
  VectorXf getComponentVelocityToWorld(Manipulator *manipulator, Name name, bool *error = false);
  VectorXf getComponentAccelerationToWorld(Manipulator *manipulator, Name name, bool *error = false);
  Pose getComponentRelativePoseToParent(Manipulator *manipulator, Name name, bool *error = false);
  Vector3f getComponentRelativePositionToParent(Manipulator *manipulator, Name name, bool *error = false);
  Matrix3f getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name, bool *error = false);
  Joint getComponentJoint(Manipulator *manipulator, Name name, bool *error = false);
  int8_t getComponentJointId(Manipulator *manipulator, Name name, bool *error = false);
  Vector3f getComponentJointAxis(Manipulator *manipulator, Name name, bool *error = false);
  float getComponentJointAngle(Manipulator *manipulator, Name name, bool *error = false);
  float getComponentJointVelocity(Manipulator *manipulator, Name name, bool *error = false);
  float getComponentJointAcceleration(Manipulator *manipulator, Name name, bool *error = false);
  Tool getComponentTool(Manipulator *manipulator, Name name, bool *error = false);
  int8_t getComponentToolId(Manipulator *manipulator, Name name, bool *error = false);
  bool getComponentToolOnOff(Manipulator *manipulator, Name name, bool *error = false);
  float getComponentToolValue(Manipulator *manipulator, Name name, bool *error = false);
  float getComponentMass(Manipulator *manipulator, Name name, bool *error = false);
  Matrix3f getComponentInertiaTensor(Manipulator *manipulator, Name name, bool *error = false);
  Pose getComponentCenterOfMassPose(Manipulator *manipulator, Name name, bool *error = false);
  Vector3f getComponentCenterOfMassPosition(Manipulator *manipulator, Name name, bool *error = false);
  Matrix3f getComponentCenterOfMassOrientation(Manipulator *manipulator, Name name, bool *error = false);
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
   
   
   
   
   
   
  