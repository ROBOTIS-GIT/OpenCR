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

#include <vector>
#include <map>

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
class OpenManipulator
{
private:
  std::map<Name, OM_MANAGER::Manipulator> manipulator_;

public:
  OpenManipulator(){};
  virtual ~OpenManipulator(){};

  void addManipulator(Name manipulator_name);

  void addWorld(Name manipulator_name,
                Name world_name,
                Name child_name,
                Vector3f world_position = Vector3f::Zero(),
                Matrix3f world_orientation = Matrix3f::Identity(3, 3));

  void addComponent(Name manipulator_name,
                    Name me_name,
                    Name parent_name,
                    Name child_name,
                    Vector3f relative_position,
                    Matrix3f relative_orientation,
                    Vector3f axis_of_rotation = Vector3f::Zero(),
                    int8_t actuator_id = -1,
                    float coefficient = 1,
                    float mass = 0.0,
                    Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
                    Vector3f center_of_mass = Vector3f::Zero());

  void addTool(Name manipulator_name,
               Name me_name,
               Name parent_name,
               Vector3f relative_position,
               Matrix3f relative_orientation,
               int8_t tool_id = -1,
               float coefficient = 1,
               float mass = 0.0,
               Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
               Vector3f center_of_mass = Vector3f::Zero());

  void addComponentChild(Name manipulator_name, Name me_name, Name child_name);
  void checkManipulatorSetting(Name manipulator_name);

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Name manipulator_name, Pose world_pose);
  void setWorldPosition(Name manipulator_name, Vector3f world_position);
  void setWorldOrientation(Name manipulator_name, Matrix3f world_orientation);
  void setWorldState(Name manipulator_name, State world_state);
  void setWorldVelocity(Name manipulator_name, VectorXf world_velocity);
  void setWorldAcceleration(Name manipulator_name, VectorXf world_acceleration);
  void setComponent(Name manipulator_name, Name name, Component component);
  void setComponentPoseToWorld(Name manipulator_name, Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name manipulator_name, Name name, Vector3f position_to_world);
  void setComponentOrientationToWorld(Name manipulator_name, Name name, Matrix3f orientation_to_wolrd);
  void setComponentStateToWorld(Name manipulator_name, Name name, State state_to_world);
  void setComponentVelocityToWorld(Name manipulator_name, Name name, VectorXf velocity);
  void setComponentAccelerationToWorld(Name manipulator_name, Name name, VectorXf accelaration);
  void setComponentJointAngle(Name manipulator_name, Name name, float angle);
  void setAllActiveJointAngle(Name manipulator_name, std::vector<float> angle_vector);
  void setComponentJointVelocity(Name manipulator_name, Name name, float angular_velocity);
  void setComponentJointAcceleration(Name manipulator_name, Name name, float angular_acceleration);
  void setComponentToolOnOff(Name manipulator_name, Name name, bool on_off);
  void setComponentToolValue(Name manipulator_name, Name name, float actuator_value);

  ///////////////////////////////Get function//////////////////////////////////
  std::map<Name, OM_MANAGER::Manipulator> getManipulator();
  OM_MANAGER::Manipulator* getManipulator(Name manipulator_name);
  int8_t getDOF(Name manipulator_name);
  int8_t getComponentSize(Name manipulator_name);
  Name getWorldName(Name manipulator_name);
  Name getWorldChildName(Name manipulator_name);
  Pose getWorldPose(Name manipulator_name);
  Vector3f getWorldPosition(Name manipulator_name);
  Matrix3f getWorldOrientation(Name manipulator_name);
  State getWorldState(Name manipulator_name);
  VectorXf getWorldVelocity(Name manipulator_name);
  VectorXf getWorldAcceleration(Name manipulator_name);
  std::map<Name, Component> getAllComponent(Name manipulator_name);
  std::map<Name, Component>::iterator getIteratorBegin(Name manipulator_name);
  std::map<Name, Component>::iterator getIteratorEnd(Name manipulator_name);
  Component getComponent(Name manipulator_name, Name name);
  Name getComponentParentName(Name manipulator_name, Name name);
  std::vector<Name> getComponentChildName(Name manipulator_name, Name name);
  Pose getComponentPoseToWorld(Name manipulator_name, Name name);
  Vector3f getComponentPositionToWorld(Name manipulator_name, Name name);
  Matrix3f getComponentOrientationToWorld(Name manipulator_name, Name name);
  State getComponentStateToWorld(Name manipulator_name, Name name);
  VectorXf getComponentVelocityToWorld(Name manipulator_name, Name name);
  VectorXf getComponentAccelerationToWorld(Name manipulator_name, Name name);
  Pose getComponentRelativePoseToParent(Name manipulator_name, Name name);
  Vector3f getComponentRelativePositionToParent(Name manipulator_name, Name name);
  Matrix3f getComponentRelativeOrientationToParent(Name manipulator_name, Name name);
  Joint getComponentJoint(Name manipulator_name, Name name);
  int8_t getComponentJointId(Name manipulator_name, Name name);
  float getComponentJointCoefficient(Name manipulator_name, Name name);
  Vector3f getComponentJointAxis(Name manipulator_name, Name name);
  float getComponentJointAngle(Name manipulator_name, Name name);
  std::vector<float> getAllJointAngle(Name manipulator_name);
  std::vector<float> getAllActiveJointAngle(Name manipulator_name);
  float getComponentJointVelocity(Name manipulator_name, Name name);
  float getComponentJointAcceleration(Name manipulator_name, Name name);
  Tool getComponentTool(Name manipulator_name, Name name);
  int8_t getComponentToolId(Name manipulator_name, Name name);
  float getComponentToolCoefficient(Name manipulator_name, Name name);
  bool getComponentToolOnOff(Name manipulator_name, Name name);
  float getComponentToolValue(Name manipulator_name, Name name);
  float getComponentMass(Name manipulator_name, Name name);
  Matrix3f getComponentInertiaTensor(Name manipulator_name, Name name);
  Vector3f getComponentCenterOfMass(Name manipulator_name, Name name);
};

class Kinematics : public OpenManipulator
{
public:
  Kinematics(){};
  ~Kinematics(){};

  virtual MatrixXf jacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name) = 0;
  // virtual void forward(OM_MANAGER::Manipulator *manipulator) = 0;
  virtual void forward(OM_MANAGER::Manipulator *manipulator, Name component_name) = 0;
  // virtual std::vector<float> inverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose) = 0;
};

#if 0
namespace MANAGER
{
///////////////////////////*initialize function*/////////////////////////////
void addWorld(OM_MANAGER::Manipulator *manipulator,
              Name world_name,
              Name child_name,
              Vector3f world_position = Vector3f::Zero(),
              Matrix3f world_orientation = Matrix3f::Identity(3, 3));

void addComponent(OM_MANAGER::Manipulator *manipulator,
                  Name me_name,
                  Name parent_name,
                  Name child_name,
                  Vector3f relative_position,
                  Matrix3f relative_orientation,
                  Vector3f axis_of_rotation = Vector3f::Zero(),
                  int8_t actuator_id = -1,
                  float coefficient = 1,
                  float mass = 0.0,
                  Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
                  Vector3f center_of_mass = Vector3f::Zero());

void addTool(OM_MANAGER::Manipulator *manipulator,
             Name me_name,
             Name parent_name,
             Vector3f relative_position,
             Matrix3f relative_orientation,
             int8_t tool_id = -1,
             float coefficient = 1,
             float mass = 0.0,
             Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
             Vector3f center_of_mass = Vector3f::Zero());

void addComponentChild(OM_MANAGER::Manipulator *manipulator, Name me_name, Name child_name);
void checkManipulatorSetting(OM_MANAGER::Manipulator *manipulator);

///////////////////////////////Set function//////////////////////////////////
void setWorldPose(OM_MANAGER::Manipulator *manipulator, Pose world_pose);
void setWorldPosition(OM_MANAGER::Manipulator *manipulator, Vector3f world_position);
void setWorldOrientation(OM_MANAGER::Manipulator *manipulator, Matrix3f world_orientation);
void setWorldState(OM_MANAGER::Manipulator *manipulator, State world_state);
void setWorldVelocity(OM_MANAGER::Manipulator *manipulator, VectorXf world_velocity);
void setWorldAcceleration(OM_MANAGER::Manipulator *manipulator, VectorXf world_acceleration);
void setComponent(OM_MANAGER::Manipulator *manipulator, Name name, Component component);
void setComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Pose pose_to_world);
void setComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Vector3f position_to_world);
void setComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd);
void setComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name, State state_to_world);
void setComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf velocity);
void setComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name, VectorXf accelaration);
void setComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name, float angle);
void setAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator, std::vector<float> angle_vector);
void setComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name, float angular_velocity);
void setComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name, float angular_acceleration);
void setComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name, bool on_off);
void setComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name, float actuator_value);

///////////////////////////////Get function//////////////////////////////////
int8_t getDOF(OM_MANAGER::Manipulator *manipulator);
int8_t getComponentSize(OM_MANAGER::Manipulator *manipulator);
Name getWorldName(OM_MANAGER::Manipulator *manipulator);
Name getWorldChildName(OM_MANAGER::Manipulator *manipulator);
Pose getWorldPose(OM_MANAGER::Manipulator *manipulator);
Vector3f getWorldPosition(OM_MANAGER::Manipulator *manipulator);
Matrix3f getWorldOrientation(OM_MANAGER::Manipulator *manipulator);
State getWorldState(OM_MANAGER::Manipulator *manipulator);
VectorXf getWorldVelocity(OM_MANAGER::Manipulator *manipulator);
VectorXf getWorldAcceleration(OM_MANAGER::Manipulator *manipulator);
std::map<Name, Component> getAllComponent(OM_MANAGER::Manipulator *manipulator);
std::map<Name, Component>::iterator getIteratorBegin(OM_MANAGER::Manipulator *manipulator);
std::map<Name, Component>::iterator getIteratorEnd(OM_MANAGER::Manipulator *manipulator);
Component getComponent(OM_MANAGER::Manipulator *manipulator, Name name);
Name getComponentParentName(OM_MANAGER::Manipulator *manipulator, Name name);
std::vector<Name> getComponentChildName(OM_MANAGER::Manipulator *manipulator, Name name);
Pose getComponentPoseToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
Vector3f getComponentPositionToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
Matrix3f getComponentOrientationToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
State getComponentStateToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
VectorXf getComponentVelocityToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
VectorXf getComponentAccelerationToWorld(OM_MANAGER::Manipulator *manipulator, Name name);
Pose getComponentRelativePoseToParent(OM_MANAGER::Manipulator *manipulator, Name name);
Vector3f getComponentRelativePositionToParent(OM_MANAGER::Manipulator *manipulator, Name name);
Matrix3f getComponentRelativeOrientationToParent(OM_MANAGER::Manipulator *manipulator, Name name);
Joint getComponentJoint(OM_MANAGER::Manipulator *manipulator, Name name);
int8_t getComponentJointId(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentJointCoefficient(OM_MANAGER::Manipulator *manipulator, Name name);
Vector3f getComponentJointAxis(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentJointAngle(OM_MANAGER::Manipulator *manipulator, Name name);
std::vector<float> getAllJointAngle(OM_MANAGER::Manipulator *manipulator);
std::vector<float> getAllActiveJointAngle(OM_MANAGER::Manipulator *manipulator);
float getComponentJointVelocity(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentJointAcceleration(OM_MANAGER::Manipulator *manipulator, Name name);
Tool getComponentTool(OM_MANAGER::Manipulator *manipulator, Name name);
int8_t getComponentToolId(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentToolCoefficient(OM_MANAGER::Manipulator *manipulator, Name name);
bool getComponentToolOnOff(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentToolValue(OM_MANAGER::Manipulator *manipulator, Name name);
float getComponentMass(OM_MANAGER::Manipulator *manipulator, Name name);
Matrix3f getComponentInertiaTensor(OM_MANAGER::Manipulator *manipulator, Name name);
Vector3f getComponentCenterOfMass(OM_MANAGER::Manipulator *manipulator, Name name);
} // namespace MANAGER

namespace KINEMATICS
{
void connectGetJacobianFunction(MatrixXf (*fp)(OM_MANAGER::Manipulator *, Name));
// void connectForward(void (*fp)(OM_MANAGER::Manipulator *manipulator));
void connectSolveForwardFunction(void (*fp)(OM_MANAGER::Manipulator *, Name));
void connectSolveInverseFunction(std::vector<float> (*fp)(OM_MANAGER::Manipulator *, Name, Pose));

MatrixXf getJacobian(OM_MANAGER::Manipulator *manipulator, Name tool_name);
void solveForward(OM_MANAGER::Manipulator *manipulator, Name component_name);
std::vector<float> solveInverse(OM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose);
} // namespace KINEMATICS

namespace ACTUATOR
{
void connectSendAllActuatorAngleFunction(bool (*fp)(std::vector<float>));
void connectSendActuatorAngleFunction(bool (*fp)(uint8_t, float));
void connectReceiveAllActuatorAngleFunction(std::vector<float> (*fp)(void));

bool sendAllActuatorAngle(std::vector<float> radian_vector);
bool sendActuatorAngle(uint8_t actuator_id, float radian);
std::vector<float> receiveAllActuatorAngle(void);
} // namespace ACTUATOR

// namespace PATH
// {
// void (*line)(void) = NULL;
// void (*arc)(void) = NULL;
// void (*custom)(void) = NULL;
// } // namespace PATH

// void connectLine(void (*fp)(void)) { PATH::line = fp; }
// void connectArc(void (*fp)(void)) { PATH::arc = fp; }
// void connectCustom(void (*fp)(void)) { PATH::custom = fp; }

#if 0 
namespace MATH
{
float sign(float number);
Eigen::Vector3f makeVector3(float v1, float v2, float v3);
Eigen::Matrix3f makeMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);
Eigen::Vector3f matrixLogarithm(Eigen::Matrix3f rotation_matrix);
Eigen::Matrix3f skewSymmetricMatrix(Eigen::Vector3f v);
Eigen::Matrix3f rodriguesRotationMatrix(Eigen::Vector3f axis, float angle);
Eigen::Matrix3f makeRotationMatrix(float rool, float pitch, float yaw);
Eigen::Matrix3f makeRotationMatrix(Eigen::Vector3f rotation_vector);
Eigen::Vector3f makeRotationVector(Eigen::Matrix3f rotation_matrix);
Vector3f positionDifference(Vector3f desired_position, Vector3f present_position);
Vector3f orientationDifference(Matrix3f desired_orientation, Matrix3f present_orientation);
VectorXf poseDifference(Vector3f desired_position, Vector3f present_position,
                        Matrix3f desired_orientation, Matrix3f present_orientation);
} // namespace MATH
#endif

#endif

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
