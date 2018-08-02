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
void (*foward)(void) = NULL;
void (*inverse)(void) = NULL;
void (*getPassiveJointAngle)(Manipulator* manipulator, bool *error = false) = NULL;
} // namespace KINEMATICS

void connectForward(void (*fp)(void)){KINEMATICS::foward = fp;}
void connectInverse(void (*fp)(void)){KINEMATICS::inverse = fp;}
void connectGetPassiveJointAngle(void (*fp)(void)){KINEMATICS::getPassiveJointAngle = fp;}

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
  void addWorld(Manipulator *manipulator, Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error);
  void addComponent(Manipulator *manipulator, Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error);
  void addTool(Manipulator *manipulator, Name me_name, Name parent_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t tool_id = -1, float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero());
  void addComponentChild(Manipulator *manipulator, Name me_name, NAME child_name, bool *error);
  void checkManipulatorSetting(Manipulator *manipulator, bool *error);

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Manipulator *manipulator, Pose world_pose, bool *error);
  void setWorldPosition(Manipulator *manipulator, Vector3f world_position, bool *error);
  void setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation, bool *error);
  void setWorldState(Manipulator *manipulator, State world_state, bool *error);
  void setWorldVelocity(Manipulator *manipulator, Vector6f world_velocity, bool *error);
  void setWorldAcceleration(Manipulator *manipulator, Vector6f world_acceleration, bool *error);
  void setComponent(Manipulator *manipulator, Name name, Component component, bool *error);
  void setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world, bool *error);
  void setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world, bool *error);
  void setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd, bool *error);
  void setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world, bool *error);
  void setComponentVelocityToWorld(Manipulator *manipulator, Name name, Vector6f velocity, bool *error);
  void setComponentAccelerationToWorld(Manipulator *manipulator, Name name, Vector6f accelaration, bool *error);
  void setComponentJointAngle(Manipulator *manipulator, Name name, float angle, bool *error);
  void setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity, bool *error);
  void setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration, bool *error);
  void setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off, bool *error);
  void setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value, bool *error);

  ///////////////////////////////Get function//////////////////////////////////
  int getDOF(Manipulator *manipulator, bool *error);
  int8_t getComponentSize(Manipulator *manipulator, bool *error);
  Name getWorldName((Manipulator *manipulator, bool *error);
  Name getWorldChildName((Manipulator *manipulator, bool *error);
  Pose getWorldPose((Manipulator *manipulator, bool *error);
  Vector3f getWorldPosition((Manipulator *manipulator, bool *error);
  Matrix3f getWorldOrientation((Manipulator *manipulator, bool *error);
  State getWorldState((Manipulator *manipulator, bool *error);
  Vector6f getWorldVelocity((Manipulator *manipulator, bool *error);
  Vector6f getWorldAcceleration((Manipulator *manipulator, bool *error);
  map<Name, Component> getAllComponent((Manipulator *manipulator, bool *error);
  Component getComponent(Manipulator *manipulator, Name name, bool *error);
  Name getComponentParentName(Manipulator *manipulator, Name name, bool *error);
  vector<NAME> getComponentChildName(Manipulator *manipulator, Name name, bool *error);
  Pose getComponentPoseToWorld(Manipulator *manipulator, Name name, bool *error);
  Vector3f getComponentPositionToWorld(Manipulator *manipulator, Name name, bool *error);
  Matrix3f getComponentOrientationToWorld(Manipulator *manipulator, Name name, bool *error);
  State getComponentStateToWorld(Manipulator *manipulator, Name name, bool *error);
  Vector6f getComponentVelocityToWorld(Manipulator *manipulator, Name name, bool *error);
  Vector6f getComponentAccelerationToWorld(Manipulator *manipulator, Name name, bool *error);
  Pose getComponentRelativePoseToParent(Manipulator *manipulator, Name name, bool *error);
  Vector3f getComponentRelativePositionToParent(Manipulator *manipulator, Name name, bool *error);
  Matrix3f getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name, bool *error);
  Joint getComponentJoint(Manipulator *manipulator, Name name, bool *error);
  int8_t getComponentJointId(Manipulator *manipulator, Name name, bool *error);
  Vector3f getComponentJointAxis(Manipulator *manipulator, Name name, bool *error);
  float getComponentJointAngle(Manipulator *manipulator, Name name, bool *error);
  float getComponentJointVelocity(Manipulator *manipulator, Name name, bool *error);
  float getComponentJointAcceleration(Manipulator *manipulator, Name name, bool *error);
  Tool getComponentTool(Manipulator *manipulator, Name name, bool *error);
  int8_t getComponentToolId(Manipulator *manipulator, Name name, bool *error);
  bool getComponentToolOnOff(Manipulator *manipulator, Name name, bool *error);
  float getComponentToolValue(Manipulator *manipulator, Name name, bool *error);
  float getComponentMass(Manipulator *manipulator, Name name, bool *error);
  Matrix3f getComponentInertiaTensor(Manipulator *manipulator, Name name, bool *error);
  Pose getComponentCenterOfMassPose(Manipulator *manipulator, Name name, bool *error);
  Vector3f getComponentCenterOfMassPosition(Manipulator *manipulator, Name name, bool *error);
  Matrix3f getComponentCenterOfMassOrientation(Manipulator *manipulator, Name name, bool *error);
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
   
   
   
   
   
   
  