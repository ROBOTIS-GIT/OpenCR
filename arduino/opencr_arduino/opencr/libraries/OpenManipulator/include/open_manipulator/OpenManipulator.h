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
typedef struct
{
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> acceleration;
  Pose pose;
} Goal;

class OpenManipulator
{
private:
  OM_MANAGER::Manipulator manipulator_;
  OM_PATH::JointTrajectory *joint_trajectory_;
  Goal previous_goal_;

  std::vector<Trajectory> start_trajectory_;
  std::vector<Trajectory> goal_trajectory_;

  Manager *manager_;
  Kinematics *kinematics_;
  Actuator *actuator_;    

  float move_time_;
  float control_time_;

  bool moving_;
  uint16_t step_cnt_;
  float present_time_;        //[s]
  float start_time_; 

  bool platform_;
  bool processing_;

  String cmd_[50];

public:
  OpenManipulator();
  virtual ~OpenManipulator();

  void initKinematics(Kinematics *kinematics);
  void initActuator(Actuator *actuator);

  void initJointTrajectory();

  void connectProcessing(uint8_t actuator_num);
  void sendAngleToProcessing(std::vector<float> joint_angle);
  void sendToolData2Processing(float value);
  String* parseDataFromProcessing(String get);

  ///////////////////////////*initialize function*/////////////////////////////
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
                    int8_t actuator_id = -1,
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

  void setComponent(Name name, Component component);
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
  void setComponentToolValue(Name name, float actuator_value);

  void setAllActiveJointAngle(std::vector<float> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();

  int8_t getComponentSize();
  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Vector3f getWorldPosition();
  Matrix3f getWorldOrientation();
  State getWorldState();
  VectorXf getWorldVelocity();
  VectorXf getWorldAcceleration();

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

  // KINEMATICS (INCLUDES VIRTUAL)
  MatrixXf jacobian(Name tool_name);
  void forward();
  void forward(Name first_component_name);
  std::vector<float> inverse(Name tool_name, Pose goal_pose);

  // ACTUATOR (INCLUDES VIRTUAL)
  void actuatorInit(const void *arg);
  void setActuatorControlMode();
  void actuatorEnable();
  void actuatorDisable();
  bool sendAllActuatorAngle(std::vector<float> radian_vector);
  bool sendMultipleActuatorAngle(std::vector<uint8_t> active_joint_id, std::vector<float> radian_vector);
  bool sendActuatorAngle(uint8_t active_joint_id, float radian);
  bool sendActuatorSignal(uint8_t active_joint_id, bool onoff);
  std::vector<float> receiveAllActuatorAngle();

  // PATH
  void setPresentTime(float present_time);
  void setMoveTime(float move_time);
  void setControlTime(float control_time);

  float getMoveTime();
  float getControlTime();

  void makeTrajectory(std::vector<Trajectory> start,
                      std::vector<Trajectory> goal);
  MatrixXf getTrajectoryCoefficient();
  void move();
  bool moving();
  void jointControl(bool flug_use_time = false);

  void setStartTrajectory(Trajectory trajectory);
  void clearStartTrajectory();
  std::vector<Trajectory> getStartTrajectory();

  void setGoalTrajectory(Trajectory trajectory);
  void clearGoalTrajectory();
  std::vector<Trajectory> getGoalTrajectory();

  // Additional Functions
  void jointMove(std::vector<float> goal_position, float move_time = 1.0f);
  bool toolMove(Name tool_name, bool onoff);
  bool toolMove(Name tool_name, float tool_value);

  void setPose(Name tool_name, Pose goal_pose, float move_time = 1.0f);
  void setMove(Name tool_name, Vector3f meter, float move_time = 1.0f);
};
} // namespace OPEN_MANIPULATOR

#endif