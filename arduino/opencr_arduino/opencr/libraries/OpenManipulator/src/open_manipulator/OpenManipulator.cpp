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

#include "../../include/open_manipulator/OpenManipulator.h"

using namespace OPEN_MANIPULATOR;

osMutexDef(om_mutex);
osMutexId(om_mutex_id);

void MUTEX::create() { om_mutex_id = osMutexCreate(osMutex(om_mutex)); }
void MUTEX::wait() { osMutexWait(om_mutex_id, osWaitForever); }
void MUTEX::release() { osMutexRelease(om_mutex_id); }

////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

OpenManipulator::OpenManipulator(uint8_t active_joint_num) : move_time_(1.0),
                                                             control_time_(0.010),
                                                             moving_(false)
{
  manager_ = new Manager();
  joint_trajectory_ = new OM_PATH::JointTrajectory(active_joint_num);

  goal_position_.reserve(active_joint_num);
  goal_velocity_.reserve(active_joint_num);
  goal_acceleration_.reserve(active_joint_num);

  start_trajectory_.reserve(active_joint_num);
  goal_trajectory_.reserve(active_joint_num);
}

OpenManipulator::~OpenManipulator()
{
}

void OpenManipulator::initKinematics(Kinematics *kinematics)
{
  kinematics_ = kinematics;
}

void OpenManipulator::initActuator(Actuator *actuator)
{
  actuator_ = actuator;
}

void OpenManipulator::addManipulator(Name manipulator_name)
{
  OM_MANAGER::Manipulator manipulator;

  manipulator_.insert(std::make_pair(manipulator_name, manipulator));
}

void OpenManipulator::addWorld(Name manipulator_name,
                               Name world_name,
                               Name child_name,
                               Vector3f world_position,
                               Matrix3f world_orientation)
{
  manipulator_.at(manipulator_name).addWorld(world_name, child_name, world_position, world_orientation);
}

void OpenManipulator::addComponent(Name manipulator_name,
                                   Name my_name,
                                   Name parent_name,
                                   Name child_name,
                                   Vector3f relative_position,
                                   Matrix3f relative_orientation,
                                   Vector3f axis_of_rotation,
                                   int8_t actuator_id,
                                   float coefficient,
                                   float mass,
                                   Matrix3f inertia_tensor,
                                   Vector3f center_of_mass)
{
  manipulator_.at(manipulator_name).addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation, actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addTool(Name manipulator_name,
                              Name my_name,
                              Name parent_name,
                              Vector3f relative_position,
                              Matrix3f relative_orientation,
                              int8_t tool_id,
                              float coefficient,
                              float mass,
                              Matrix3f inertia_tensor,
                              Vector3f center_of_mass)
{
  manipulator_.at(manipulator_name).addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addComponentChild(Name manipulator_name, Name my_name, Name child_name)
{
  manipulator_.at(manipulator_name).addComponentChild(my_name, child_name);
}

void OpenManipulator::checkManipulatorSetting(Name manipulator_name)
{
  manipulator_.at(manipulator_name).checkManipulatorSetting();
}

void OpenManipulator::setWorldPose(Name manipulator_name, Pose world_pose)
{
  manipulator_.at(manipulator_name).setWorldPose(world_pose);
}

void OpenManipulator::setWorldPosition(Name manipulator_name, Vector3f world_position)
{
  manipulator_.at(manipulator_name).setWorldPosition(world_position);
}

void OpenManipulator::setWorldOrientation(Name manipulator_name, Matrix3f world_orientation)
{
  manipulator_.at(manipulator_name).setWorldOrientation(world_orientation);
}

void OpenManipulator::setWorldState(Name manipulator_name, State world_state)
{
  manipulator_.at(manipulator_name).setWorldState(world_state);
}

void OpenManipulator::setWorldVelocity(Name manipulator_name, VectorXf world_velocity)
{
  manipulator_.at(manipulator_name).setWorldVelocity(world_velocity);
}

void OpenManipulator::setWorldAcceleration(Name manipulator_name, VectorXf world_acceleration)
{
  manipulator_.at(manipulator_name).setWorldAcceleration(world_acceleration);
}

void OpenManipulator::setComponent(Name manipulator_name, Name name, Component component)
{
  manipulator_.at(manipulator_name).setComponent(name, component);
}

void OpenManipulator::setComponentPoseToWorld(Name manipulator_name, Name name, Pose pose_to_world)
{
  manipulator_.at(manipulator_name).setComponentPoseToWorld(name, pose_to_world);
}

void OpenManipulator::setComponentPositionToWorld(Name manipulator_name, Name name, Vector3f position_to_world)
{
  manipulator_.at(manipulator_name).setComponentPositionToWorld(name, position_to_world);
}

void OpenManipulator::setComponentOrientationToWorld(Name manipulator_name, Name name, Matrix3f orientation_to_wolrd)
{
  manipulator_.at(manipulator_name).setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void OpenManipulator::setComponentStateToWorld(Name manipulator_name, Name name, State state_to_world)
{
  manipulator_.at(manipulator_name).setComponentStateToWorld(name, state_to_world);
}

void OpenManipulator::setComponentVelocityToWorld(Name manipulator_name, Name name, VectorXf velocity)
{
  manipulator_.at(manipulator_name).setComponentVelocityToWorld(name, velocity);
}

void OpenManipulator::setComponentAccelerationToWorld(Name manipulator_name, Name name, VectorXf accelaration)
{
  manipulator_.at(manipulator_name).setComponentAccelerationToWorld(name, accelaration);
}

void OpenManipulator::setComponentJointAngle(Name manipulator_name, Name name, float angle)
{
  manipulator_.at(manipulator_name).setComponentJointAngle(name, angle);
}

void OpenManipulator::setAllActiveJointAngle(Name manipulator_name, std::vector<float> angle_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) != -1)
    {
      manipulator_.at(manipulator_name).setComponentJointAngle(it->first, angle_vector.at(index));
      index++;
    }
  }
}

void OpenManipulator::setComponentJointVelocity(Name manipulator_name, Name name, float angular_velocity)
{
  manipulator_.at(manipulator_name).setComponentJointVelocity(name, angular_velocity);
}

void OpenManipulator::setComponentJointAcceleration(Name manipulator_name, Name name, float angular_acceleration)
{
  manipulator_.at(manipulator_name).setComponentJointAcceleration(name, angular_acceleration);
}

void OpenManipulator::setComponentToolOnOff(Name manipulator_name, Name name, bool on_off)
{
  manipulator_.at(manipulator_name).setComponentToolOnOff(name, on_off);
}

void OpenManipulator::setComponentToolValue(Name manipulator_name, Name name, float actuator_value)
{
  manipulator_.at(manipulator_name).setComponentToolValue(name, actuator_value);
}

int8_t OpenManipulator::getDOF(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getDOF();
}

int8_t OpenManipulator::getComponentSize(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getComponentSize();
}

Name OpenManipulator::getWorldName(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldName();
}

Name OpenManipulator::getWorldChildName(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldChildName();
}

Pose OpenManipulator::getWorldPose(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldPose();
}

Vector3f OpenManipulator::getWorldPosition(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldPosition();
}

Matrix3f OpenManipulator::getWorldOrientation(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldOrientation();
}

State OpenManipulator::getWorldState(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldState();
}

VectorXf OpenManipulator::getWorldVelocity(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldVelocity();
}

VectorXf OpenManipulator::getWorldAcceleration(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getWorldAcceleration();
}

std::map<Name, Component> OpenManipulator::getAllComponent(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getAllComponent();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorBegin(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getIteratorBegin();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorEnd(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getIteratorEnd();
}

Component OpenManipulator::getComponent(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponent(name);
}

Name OpenManipulator::getComponentParentName(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentParentName(name);
}

std::vector<Name> OpenManipulator::getComponentChildName(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentChildName(name);
}

Pose OpenManipulator::getComponentPoseToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentPoseToWorld(name);
}

Vector3f OpenManipulator::getComponentPositionToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentPositionToWorld(name);
}

Matrix3f OpenManipulator::getComponentOrientationToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentOrientationToWorld(name);
}

State OpenManipulator::getComponentStateToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentStateToWorld(name);
}

VectorXf OpenManipulator::getComponentVelocityToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentVelocityToWorld(name);
}

VectorXf OpenManipulator::getComponentAccelerationToWorld(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentAccelerationToWorld(name);
}

Pose OpenManipulator::getComponentRelativePoseToParent(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentRelativePoseToParent(name);
}

Vector3f OpenManipulator::getComponentRelativePositionToParent(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentRelativePositionToParent(name);
}

Matrix3f OpenManipulator::getComponentRelativeOrientationToParent(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentRelativeOrientationToParent(name);
}

Joint OpenManipulator::getComponentJoint(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJoint(name);
}

int8_t OpenManipulator::getComponentJointId(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointId(name);
}

float OpenManipulator::getComponentJointCoefficient(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointCoefficient(name);
}

Vector3f OpenManipulator::getComponentJointAxis(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointAxis(name);
}

float OpenManipulator::getComponentJointAngle(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointAngle(name);
}

std::vector<float> OpenManipulator::getAllJointAngle(Name manipulator_name)
{
  std::vector<float> result_vector;
  std::map<Name, Component>::iterator it;
  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentToolId(it->first) < 0)
    {
      result_vector.push_back(manipulator_.at(manipulator_name).getComponentJointAngle(it->first));
    }
  }
  return result_vector;
}

std::vector<float> OpenManipulator::getAllActiveJointAngle(Name manipulator_name)
{
  std::vector<float> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) != -1)
    {
      result_vector.push_back(manipulator_.at(manipulator_name).getComponentJointAngle(it->first));
    }
  }
  return result_vector;
}

uint8_t OpenManipulator::getNumberOfActiveJoint(Name manipulator_name)
{
  uint8_t active_joint_num = 0;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) != -1)
    {
      active_joint_num++;
    }
  }
  return active_joint_num;
}

float OpenManipulator::getComponentJointVelocity(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointVelocity(name);
}

float OpenManipulator::getComponentJointAcceleration(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentJointAcceleration(name);
}

Tool OpenManipulator::getComponentTool(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentTool(name);
}

int8_t OpenManipulator::getComponentToolId(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentToolId(name);
}

float OpenManipulator::getComponentToolCoefficient(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentToolCoefficient(name);
}

bool OpenManipulator::getComponentToolOnOff(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentToolOnOff(name);
}

float OpenManipulator::getComponentToolValue(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentToolValue(name);
}

float OpenManipulator::getComponentMass(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentMass(name);
}

Matrix3f OpenManipulator::getComponentInertiaTensor(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentInertiaTensor(name);
}

Vector3f OpenManipulator::getComponentCenterOfMass(Name manipulator_name, Name name)
{
  return manipulator_.at(manipulator_name).getComponentCenterOfMass(name);
}

MatrixXf OpenManipulator::jacobian(Name manipulator_name, Name tool_name)
{
  return kinematics_->jacobian(&manipulator_.at(manipulator_name), tool_name);
}

void OpenManipulator::forward(Name manipulator_name)
{
  return kinematics_->forward(&manipulator_.at(manipulator_name));
}

void OpenManipulator::forward(Name manipulator_name, Name component_name)
{
  return kinematics_->forward(&manipulator_.at(manipulator_name), component_name);
}

std::vector<float> OpenManipulator::inverse(Name manipulator_name, Name tool_name, Pose target_pose)
{
  return kinematics_->inverse(&manipulator_.at(manipulator_name), tool_name, target_pose);
}

bool OpenManipulator::sendAllActuatorAngle(std::vector<float> radian_vector)
{
  return actuator_->sendAllActuatorAngle(radian_vector);
}

bool OpenManipulator::sendActuatorAngle(uint8_t actuator_id, float radian)
{
  return actuator_->sendActuatorAngle(actuator_id, radian);
}

std::vector<float> OpenManipulator::receiveAllActuatorAngle(void)
{
  return actuator_->receiveAllActuatorAngle();
}

void OpenManipulator::actuatorInit(const void *arg)
{
  return actuator_->initActuator(arg);
}

void OpenManipulator::actuatorEnable()
{
  return actuator_->Enable();
}

void OpenManipulator::actuatorDisable()
{
  return actuator_->Disable();
}

void OpenManipulator::setMoveTime(float move_time)
{
  move_time_ = move_time;
}

void OpenManipulator::setControlTime(float control_time)
{
  control_time_ = control_time;
}

float OpenManipulator::getMoveTime()
{
  return move_time_;
}

float OpenManipulator::getControlTime()
{
  return control_time_;
}

void OpenManipulator::makeTrajectory(std::vector<Trajectory> start,
                                     std::vector<Trajectory> goal)
{
  joint_trajectory_->init(start, goal, move_time_, control_time_);
}

MatrixXf OpenManipulator::getTrajectoryCoefficient()
{
  joint_trajectory_->getCoefficient();
}

void OpenManipulator::move()
{
  moving_ = true;
}

void OpenManipulator::setStartTrajectory(Trajectory trajectory)
{
  start_trajectory_.push_back(trajectory);
}

void OpenManipulator::clearStartTrajectory()
{
  start_trajectory_.clear();
}

std::vector<Trajectory> OpenManipulator::getStartTrajectory()
{
  return start_trajectory_;
}

void OpenManipulator::setGoalTrajectory(Trajectory trajectory)
{
  goal_trajectory_.push_back(trajectory);
}

void OpenManipulator::clearGoalTrajectory()
{
  goal_trajectory_.clear();
}

std::vector<Trajectory> OpenManipulator::getGoalTrajectory()
{
  return goal_trajectory_;
}

void OpenManipulator::jointControl()
{
  uint16_t step_time = uint16_t(floor(move_time_ / control_time_) + 1.0);
  float tick_time = 0;
  static uint16_t step_cnt = 0;

  goal_position_.clear();
  goal_velocity_.clear();
  goal_acceleration_.clear();

  if (moving_)
  {
    if (step_cnt < step_time)
    {
      tick_time = control_time_ * step_cnt;

      goal_position_ = joint_trajectory_->getPosition(tick_time);
      goal_velocity_ = joint_trajectory_->getVelocity(tick_time);
      goal_acceleration_ = joint_trajectory_->getAcceleration(tick_time);

      LOG::INFO("path : ", goal_position_.at(0));

      // dxl.setAngle(goal_position);

      step_cnt++;
    }
    else
    {
      step_cnt = 0;
      moving_ = false;
    }
  }
}

