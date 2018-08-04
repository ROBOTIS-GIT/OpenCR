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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef OMAPI_H_
#define OMAPI_H_

#include <RTOS.h>
#include <Eigen.h>

#include "OMManager.h"
#include "OMMath.h"
#include "OMDebug.h"

#if 0
osMutexDef(om_mutex);
osMutexId(om_mutex_id);

namespace MUTEX
{
void create() { om_mutex_id = osMutexCreate(osMutex(om_mutex)); }
void wait() { osMutexWait(om_mutex_id, osWaitForever); }
void release() { osMutexRelease(om_mutex_id); }
} // namespace MUTEX
#endif

namespace OPEN_MANIPULATOR
{
#if 0
// Connect functions
namespace ACTUATOR
{
bool (*setAllJointAngle)(float *) = NULL;
bool (*setJointAngle)(uint8_t, float) = NULL;
float *(*getAngle)() = NULL;
} // namespace ACTUATOR

void connectSetAllJointAngle(bool (*fp)(float *)) { ACTUATOR::setAllJointAngle = fp; }
void connectSetJointAngle(bool (*fp)(uint8_t, float)) { ACTUATOR::setJointAngle = fp; }
void connectGetAngle(float *(*fp)()) { ACTUATOR::getAngle = fp; }

namespace KINEMATICS
{
void (*foward)(Manipulator *, bool *) = NULL;
void (*inverse)(Manipulator *, Name, Pose, bool *) = NULL;
void (*getPassiveJointAngle)(Manipulator *, bool *) = NULL;
} // namespace KINEMATICS

void connectForward(void (*fp)(Manipulator *, bool *)) { KINEMATICS::foward = fp; }
void connectInverse(void (*fp)(Manipulator *, Name, Pose, bool *)) { KINEMATICS::inverse = fp; }
void connectGetPassiveJointAngle(void (*fp)(Manipulator *, bool *)) { KINEMATICS::getPassiveJointAngle = fp; }

namespace PATH
{
void (*line)(void) = NULL;
void (*arc)(void) = NULL;
void (*custom)(void) = NULL;
} // namespace PATH

void connectLine(void (*fp)(void)) { PATH::line = fp; }
void connectArc(void (*fp)(void)) { PATH::arc = fp; }
void connectCustom(void (*fp)(void)) { PATH::custom = fp; }

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
} // namespace MATH
#endif

// namespace Manipulator
namespace MANAGER
{
///////////////////////////*initialize function*/////////////////////////////
void addWorld(Manipulator *manipulator,
              Name world_name,
              Name child_name,
              Vector3f world_position = ZERO_VECTOR,
              Matrix3f world_orientation = IDENTITY_MATRIX);

void addComponent(Manipulator *manipulator,
                  Name me_name,
                  Name parent_name,
                  Name child_name,
                  Vector3f relative_position,
                  Matrix3f relative_orientation,
                  int8_t actuator_id = -1,
                  Vector3f axis_of_rotation = ZERO_VECTOR,
                  float mass = 0.0,
                  Matrix3f inertia_tensor = IDENTITY_MATRIX,
                  Vector3f center_of_mass = ZERO_VECTOR);

void addTool(Manipulator *manipulator,
             Name me_name,
             Name parent_name,
             Vector3f relative_position,
             Matrix3f relative_orientation,
             int8_t tool_id = -1,
             float mass = 0.0,
             Matrix3f inertia_tensor = IDENTITY_MATRIX,
             Vector3f center_of_mass = ZERO_VECTOR);

void addComponentChild(Manipulator *manipulator, Name me_name, Name child_name);
void checkManipulatorSetting(Manipulator *manipulator);

///////////////////////////////Set function//////////////////////////////////
void setWorldPose(Manipulator *manipulator, Pose world_pose);
void setWorldPosition(Manipulator *manipulator, Vector3f world_position);
void setWorldOrientation(Manipulator *manipulator, Matrix3f world_orientation);
void setWorldState(Manipulator *manipulator, State world_state);
void setWorldVelocity(Manipulator *manipulator, VectorXf world_velocity);
void setWorldAcceleration(Manipulator *manipulator, VectorXf world_acceleration);
void setComponent(Manipulator *manipulator, Name name, Component component);
void setComponentPoseToWorld(Manipulator *manipulator, Name name, Pose pose_to_world);
void setComponentPositionToWorld(Manipulator *manipulator, Name name, Vector3f position_to_world);
void setComponentOrientationToWorld(Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd);
void setComponentStateToWorld(Manipulator *manipulator, Name name, State state_to_world);
void setComponentVelocityToWorld(Manipulator *manipulator, Name name, VectorXf velocity);
void setComponentAccelerationToWorld(Manipulator *manipulator, Name name, VectorXf accelaration);
void setComponentJointAngle(Manipulator *manipulator, Name name, float angle);
void setComponentJointVelocity(Manipulator *manipulator, Name name, float angular_velocity);
void setComponentJointAcceleration(Manipulator *manipulator, Name name, float angular_acceleration);
void setComponentToolOnOff(Manipulator *manipulator, Name name, bool on_off);
void setComponentToolValue(Manipulator *manipulator, Name name, float actuator_value);

///////////////////////////////Get function//////////////////////////////////
int8_t getDOF(Manipulator *manipulator);
int8_t getComponentSize(Manipulator *manipulator);
Name getWorldName(Manipulator *manipulator);
Name getWorldChildName(Manipulator *manipulator);
Pose getWorldPose(Manipulator *manipulator);
Vector3f getWorldPosition(Manipulator *manipulator);
Matrix3f getWorldOrientation(Manipulator *manipulator);
State getWorldState(Manipulator *manipulator);
VectorXf getWorldVelocity(Manipulator *manipulator);
VectorXf getWorldAcceleration(Manipulator *manipulator);

std::map<Name, Component> getAllComponent(Manipulator *manipulator);
std::map<Name, Component>::iterator getIteratorBegin(Manipulator *manipulator);
std::map<Name, Component>::iterator getIteratorEnd(Manipulator *manipulator);

Component getComponent(Manipulator *manipulator, Name name);
Name getComponentParentName(Manipulator *manipulator, Name name);
std::vector<Name> getComponentChildName(Manipulator *manipulator, Name name);
Pose getComponentPoseToWorld(Manipulator *manipulator, Name name);
Vector3f getComponentPositionToWorld(Manipulator *manipulator, Name name);
Matrix3f getComponentOrientationToWorld(Manipulator *manipulator, Name name);
State getComponentStateToWorld(Manipulator *manipulator, Name name);
VectorXf getComponentVelocityToWorld(Manipulator *manipulator, Name name);
VectorXf getComponentAccelerationToWorld(Manipulator *manipulator, Name name);
Pose getComponentRelativePoseToParent(Manipulator *manipulator, Name name);
Vector3f getComponentRelativePositionToParent(Manipulator *manipulator, Name name);
Matrix3f getComponentRelativeOrientationToParent(Manipulator *manipulator, Name name);
Joint getComponentJoint(Manipulator *manipulator, Name name);
int8_t getComponentJointId(Manipulator *manipulator, Name name);
Vector3f getComponentJointAxis(Manipulator *manipulator, Name name);
float getComponentJointAngle(Manipulator *manipulator, Name name);
float getComponentJointVelocity(Manipulator *manipulator, Name name);
float getComponentJointAcceleration(Manipulator *manipulator, Name name);
Tool getComponentTool(Manipulator *manipulator, Name name);
int8_t getComponentToolId(Manipulator *manipulator, Name name);
bool getComponentToolOnOff(Manipulator *manipulator, Name name);
float getComponentToolValue(Manipulator *manipulator, Name name);
float getComponentMass(Manipulator *manipulator, Name name);
Matrix3f getComponentInertiaTensor(Manipulator *manipulator, Name name);
Vector3f getComponentCenterOfMass(Manipulator *manipulator, Name name);
} // namespace MANAGER

#if 0
// Thread
void Thread_Robot_State(void const *argument)
{
  (void)argument;

  LOG::init();
  MUTEX::create();

  for (;;)
  {
    KINEMATICS::inverse();

    // MUTEX::wait();
    //   float* angle_ptr = ACTUATOR::getAngle();
    // MUTEX::release();
    //   LOG::INFO("angle : " + String(angle_ptr[0]));

    osDelay(10);
  }
}
#endif

} // namespace OPEN_MANIPULATOR
#endif