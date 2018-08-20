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

#ifndef OPEN_MANIPULATOR_H_
#define OPEN_MANIPULATOR_H_

#include "OMAPI.h"
#include "OMBridge.h"

#include <algorithm> // for sort()
#include <RTOS.h>

using namespace Eigen;

namespace MUTEX
{
void create();
void wait();
void release();
} // namespace MUTEX

namespace THREAD
{
void Robot_State(void const *argument);
void Actuator_Control(void const *argument);
} // namespace THREAD

namespace OPEN_MANIPULATOR
{
class OpenManipulator
{
private:
  std::map<Name, OM_MANAGER::Manipulator> manipulator_;

  Manager *manager_;
  Kinematics *kinematics_;
  Actuator *actuator_;
  OM_PATH::JointTrajectory *joint_trajectory_;

  std::vector<float> goal_position_;
  std::vector<float> goal_velocity_;
  std::vector<float> goal_acceleration_;

  std::vector<Trajectory> start_trajectory_;
  std::vector<Trajectory> goal_trajectory_;

  float move_time_;
  float control_time_;

  bool moving_;

  bool platform_;
  bool processing_;

  String cmd_[50];

public:
  OpenManipulator(uint8_t active_joint_num);
  virtual ~OpenManipulator();

  void initKinematics(Kinematics *kinematics);
  void initActuator(Actuator *actuator);

  void connectProcessing(uint8_t actuator_num);
  void sendAngleToProcessing(std::vector<float> joint_angle);
  String* parseDataFromProcessing(String get);

  void addManipulator(Name manipulator_name);

  ///////////////////////////*initialize function*/////////////////////////////
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
  void setComponentJointVelocity(Name manipulator_name, Name name, float angular_velocity);
  void setComponentJointAcceleration(Name manipulator_name, Name name, float angular_acceleration);
  void setComponentToolOnOff(Name manipulator_name, Name name, bool on_off);
  void setComponentToolValue(Name manipulator_name, Name name, float actuator_value);

  void setAllActiveJointAngle(Name manipulator_name, std::vector<float> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////
  
  OM_MANAGER::Manipulator getManipulator(Name manipulator_name);
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

  std::vector<float> getAllJointAngle(Name manipulator_name);
  std::vector<float> getAllActiveJointAngle(Name manipulator_name);
  std::vector<uint8_t> getAllActiveJointID(Name manipulator_name);

  // KINEMATICS (INCLUDES VIRTUAL)
  MatrixXf jacobian(Name manipulator_name, Name tool_name);
  void forward(Name manipulator_name);
  void forward(Name manipulator_name, Name component_name);
  std::vector<float> inverse(Name manipulator_name, Name tool_name, Pose goal_pose);

  // ACTUATOR (INCLUDES VIRTUAL)
  void actuatorInit(const void *arg);
  void actuatorEnable();
  void actuatorDisable();
  bool sendAllActuatorAngle(Name manipulator_name, std::vector<float> radian_vector);
  bool sendMultipleActuatorAngle(Name manipulator_name, std::vector<uint8_t> active_joint_id, std::vector<float> radian_vector);
  bool sendActuatorAngle(Name manipulator_name, uint8_t active_joint_id, float radian);
  bool sendActuatorSignal(uint8_t active_joint_id, bool onoff);
  std::vector<float> receiveAllActuatorAngle(Name manipulator_name);

  // PATH
  void setMoveTime(float move_time);
  void setControlTime(float control_time);

  float getMoveTime();
  float getControlTime();

  void makeTrajectory(std::vector<Trajectory> start,
                      std::vector<Trajectory> goal);
  MatrixXf getTrajectoryCoefficient();
  void move();
  bool moving();
  void jointControl(Name manipulator_name);

  void setStartTrajectory(Trajectory trajectory);
  void clearStartTrajectory();
  std::vector<Trajectory> getStartTrajectory();

  void setGoalTrajectory(Trajectory trajectory);
  void clearGoalTrajectory();
  std::vector<Trajectory> getGoalTrajectory();

  // Additional Functions
  void jointMove(Name manipulator_name, std::vector<float> goal_position, float move_time = 1.0f);
  bool toolMove(Name manipulator_name, Name tool_name, bool onoff);
  bool toolMove(Name manipulator_name, Name tool_name, float tool_value);

  void setPose(Name manipulator_name, Name tool_name, Pose goal_pose, float move_time = 1.0f);
  void setMove(Name manipulator_name, Name tool_name, Vector3f meter, float move_time = 1.0f);
};
} // namespace OPEN_MANIPULATOR

#endif