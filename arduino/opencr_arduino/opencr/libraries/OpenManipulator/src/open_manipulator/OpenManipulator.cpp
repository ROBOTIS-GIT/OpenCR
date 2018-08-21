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
using namespace Eigen;

osMutexDef(om_mutex);
osMutexId(om_mutex_id);

void MUTEX::create() { om_mutex_id = osMutexCreate(osMutex(om_mutex)); }
void MUTEX::wait() { osMutexWait(om_mutex_id, osWaitForever); }
void MUTEX::release() { osMutexRelease(om_mutex_id); }

////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

OpenManipulator::OpenManipulator() : move_time_(1.0f),
                                     control_time_(0.010f),
                                     step_cnt_(0),
                                     moving_(false),
                                     platform_(false),
                                     processing_(false)
{
  manager_ = new Manager();
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
  platform_ = true;
}

void OpenManipulator::initJointTrajectory(Name manipulator_name)
{ 
  OM_PATH::JointTrajectory joint_trajectory(manipulator_.at(manipulator_name).getDOF());
  joint_trajectory_.insert(std::make_pair(manipulator_name, joint_trajectory));

  std::vector<float> empty_v;

  for (uint8_t index = 0; index < manipulator_.at(manipulator_name).getDOF(); index++)
    empty_v.push_back(0.0);

  Goal goal;
  goal.position = receiveAllActuatorAngle(manipulator_name);
  goal.velocity = empty_v;
  goal.acceleration = empty_v;

  previous_goal_.insert(std::make_pair(manipulator_name, goal));

  start_trajectory_.reserve(manipulator_.at(manipulator_name).getDOF());
  goal_trajectory_.reserve(manipulator_.at(manipulator_name).getDOF());
}

void OpenManipulator::connectProcessing(uint8_t actuator_num)
{
  OM_PROCESSING::initProcessing((int8_t)(actuator_num));
  processing_ = true;
}

void OpenManipulator::sendAngleToProcessing(std::vector<float> joint_angle)
{
  OM_PROCESSING::sendAngle2Processing(joint_angle);
}

void OpenManipulator::sendToolData2Processing(float value)
{
  OM_PROCESSING::sendToolData2Processing(value);
}

String *OpenManipulator::parseDataFromProcessing(String get)
{
  get.trim();
  OM_PROCESSING::split(get, ',', cmd_);

  return cmd_;
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

void OpenManipulator::setAllActiveJointAngle(Name manipulator_name, std::vector<float> angle_vector)
{
  manipulator_.at(manipulator_name).setAllActiveJointAngle(angle_vector);
}

OM_MANAGER::Manipulator OpenManipulator::getManipulator(Name manipulator_name)
{
  return manipulator_.at(manipulator_name);
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

std::vector<float> OpenManipulator::getAllJointAngle(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getAllJointAngle();
}

std::vector<float> OpenManipulator::getAllActiveJointAngle(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getAllActiveJointAngle();
}

std::vector<uint8_t> OpenManipulator::getAllActiveJointID(Name manipulator_name)
{
  return manipulator_.at(manipulator_name).getAllActiveJointID();
}

// KINEMATICS

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

std::vector<float> OpenManipulator::inverse(Name manipulator_name, Name tool_name, Pose goal_pose)
{
  return kinematics_->inverse(&manipulator_.at(manipulator_name), tool_name, goal_pose);
}

// ACTUATOR
bool OpenManipulator::sendAllActuatorAngle(Name manipulator_name, std::vector<float> radian_vector)
{
  std::vector<float> calc_angle;
  std::map<Name, Component>::iterator it;

  uint8_t index = 0;
  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) != -1)
    {
      calc_angle.push_back(radian_vector.at(index++) * manipulator_.at(manipulator_name).getComponentJointCoefficient(it->first));
    }
  }

  return actuator_->sendAllActuatorAngle(calc_angle);
}

bool OpenManipulator::sendMultipleActuatorAngle(Name manipulator_name, std::vector<uint8_t> active_joint_id, std::vector<float> radian_vector)
{
  std::vector<float> calc_angle;
  std::map<Name, Component>::iterator it;

  for (uint8_t index = 0; index < active_joint_id.size(); index++)
  {
    for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
    {
      if (active_joint_id.at(index) == manipulator_.at(manipulator_name).getComponentJointId(it->first))
      {
        calc_angle.push_back(radian_vector.at(index) * manipulator_.at(manipulator_name).getComponentJointCoefficient(it->first));
        break;
      }
    }
  }

  return actuator_->sendMultipleActuatorAngle(active_joint_id, calc_angle);
}

bool OpenManipulator::sendActuatorAngle(Name manipulator_name, uint8_t active_joint_id, float radian)
{
  float calc_angle;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) == active_joint_id)
    {
      calc_angle = radian * manipulator_.at(manipulator_name).getComponentJointCoefficient(it->first);
    }
  }

  return actuator_->sendActuatorAngle(active_joint_id, calc_angle);
}

bool OpenManipulator::sendActuatorSignal(uint8_t active_joint_id, bool onoff)
{
  return actuator_->sendActuatorSignal(active_joint_id, onoff);
}

std::vector<float> OpenManipulator::receiveAllActuatorAngle(Name manipulator_name)
{
  std::vector<float> angles = actuator_->receiveAllActuatorAngle();
  std::vector<uint8_t> active_joint_id = manipulator_.at(manipulator_name).getAllActiveJointID();
  std::vector<uint8_t> sorted_id = active_joint_id;

  std::vector<float> sorted_angle_vector;
  sorted_angle_vector.reserve(active_joint_id.size());

  float sorted_angle_array[angles.size()];

  std::sort(sorted_id.begin(), sorted_id.end());
  for (uint8_t i = 0; i < sorted_id.size(); i++)
  {
    for (uint8_t j = 0; j < active_joint_id.size(); j++)
    {
      if (sorted_id.at(i) == active_joint_id.at(j))
      {
        sorted_angle_array[j] = angles.at(i);
        break;
      }
    }
  }

  for (uint8_t index = 0; index < active_joint_id.size(); index++)
    sorted_angle_vector.push_back(sorted_angle_array[index]);

  std::vector<float> calc_sorted_angle;
  std::map<Name, Component>::iterator it;

  uint8_t index = 0;
  for (it = manipulator_.at(manipulator_name).getIteratorBegin(); it != manipulator_.at(manipulator_name).getIteratorEnd(); it++)
  {
    if (manipulator_.at(manipulator_name).getComponentJointId(it->first) != -1)
    {
      calc_sorted_angle.push_back(sorted_angle_vector.at(index++) / manipulator_.at(manipulator_name).getComponentJointCoefficient(it->first));
    }
  }

  return calc_sorted_angle;
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

// JOINT TRAJECTORY

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

void OpenManipulator::makeTrajectory(Name manipulator_name,
                                     std::vector<Trajectory> start,
                                     std::vector<Trajectory> goal)
{
  joint_trajectory_.at(manipulator_name).init(start, goal, move_time_, control_time_);
}

MatrixXf OpenManipulator::getTrajectoryCoefficient(Name manipulator_name)
{
  return joint_trajectory_.at(manipulator_name).getCoefficient();
}

void OpenManipulator::move()
{
  moving_ = true;
  step_cnt_ = 0;
}

bool OpenManipulator::moving()
{
  return moving_;
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

void OpenManipulator::jointControl(Name manipulator_name)
{
  uint16_t step_time = uint16_t(floor(move_time_ / control_time_) + 1.0);
  float tick_time = 0;

  std::vector<float> goal_position;
  std::vector<float> goal_velocity;
  std::vector<float> goal_acceleration;

  goal_position.reserve(manipulator_.at(manipulator_name).getDOF());
  goal_velocity.reserve(manipulator_.at(manipulator_name).getDOF());
  goal_acceleration.reserve(manipulator_.at(manipulator_name).getDOF());

  if (moving_)
  {
    if (step_cnt_ < step_time)
    {
      tick_time = control_time_ * step_cnt_;

      goal_position = joint_trajectory_.at(manipulator_name).getPosition(tick_time);
      goal_velocity = joint_trajectory_.at(manipulator_name).getVelocity(tick_time);
      goal_acceleration = joint_trajectory_.at(manipulator_name).getAcceleration(tick_time);

      if (platform_)
        sendMultipleActuatorAngle(manipulator_name, manipulator_.at(manipulator_name).getAllActiveJointID(), goal_position);

      if (processing_)
      {
        OM_PROCESSING::sendAngle2Processing(goal_position);
        if (platform_ == false)
          manipulator_.at(manipulator_name).setAllActiveJointAngle(goal_position);
      }
        
      previous_goal_.at(manipulator_name).position = goal_position;
      previous_goal_.at(manipulator_name).velocity = goal_velocity;
      previous_goal_.at(manipulator_name).acceleration = goal_acceleration;

      step_cnt_++;
    }
    else
    {
      step_cnt_ = 0;
      moving_ = false;
    }
  }
}

void OpenManipulator::jointMove(Name manipulator_name, std::vector<float> goal_position, float move_time)
{
  Trajectory start;
  Trajectory goal;

  std::vector<float> present_position = previous_goal_.at(manipulator_name).position;
  std::vector<float> present_velocity = previous_goal_.at(manipulator_name).velocity;
  std::vector<float> present_acceleration = previous_goal_.at(manipulator_name).acceleration;

  start_trajectory_.clear();
  goal_trajectory_.clear();

  for (uint8_t index = 0; index < manipulator_.at(manipulator_name).getDOF(); index++)
  {
    start.position = present_position.at(index);
    start.velocity = present_velocity.at(index);
    start.acceleration = present_acceleration.at(index);

    start_trajectory_.push_back(start);
  
    goal.position = goal_position.at(index);
    goal.velocity = 0.0f;
    goal.acceleration = 0.0f;

    goal_trajectory_.push_back(goal);
  }

  setMoveTime(move_time);
  makeTrajectory(manipulator_name, start_trajectory_, goal_trajectory_);
  move();
}

bool OpenManipulator::toolMove(Name manipulator_name, Name tool_name, bool onoff)
{
  manipulator_.at(manipulator_name).setComponentToolOnOff(tool_name, onoff);

  if (platform_)
    actuator_->sendActuatorSignal(manipulator_.at(manipulator_name).getComponentToolId(tool_name), onoff);

  if (processing_)
    OM_PROCESSING::sendToolData2Processing(onoff);

  return true;
}

bool OpenManipulator::toolMove(Name manipulator_name, Name tool_name, float tool_value)
{
  float calc_value = tool_value * manipulator_.at(manipulator_name).getComponentToolCoefficient(tool_name);

  manipulator_.at(manipulator_name).setComponentToolValue(tool_name, calc_value);

  if (platform_)
    actuator_->sendActuatorAngle(manipulator_.at(manipulator_name).getComponentToolId(tool_name), calc_value);

  if (processing_)
    OM_PROCESSING::sendToolData2Processing(tool_value);

  return true;
}

void OpenManipulator::setPose(Name manipulator_name, Name tool_name, Pose goal_pose, float move_time)
{
  std::vector<float> goal_position = kinematics_->inverse(&manipulator_.at(manipulator_name), tool_name, goal_pose);

  jointMove(manipulator_name, goal_position, move_time);
}

void OpenManipulator::setMove(Name manipulator_name, Name tool_name, Vector3f meter, float move_time)
{
  setAllActiveJointAngle(manipulator_name, previous_goal_.at(manipulator_name).position);
  forward(manipulator_name, manipulator_.at(manipulator_name).getWorldChildName());

  previous_goal_.at(manipulator_name).pose = manipulator_.at(manipulator_name).getComponentPoseToWorld(tool_name);
  
  setAllActiveJointAngle(manipulator_name, receiveAllActuatorAngle(manipulator_name));
  forward(manipulator_name, manipulator_.at(manipulator_name).getWorldChildName());

  Vector3f present_position_to_world = previous_goal_.at(manipulator_name).pose.position;
  Matrix3f present_orientation_to_world = previous_goal_.at(manipulator_name).pose.orientation;

  Vector3f goal_position_to_world = present_position_to_world + meter;

  Pose goal_pose;
  goal_pose.position = goal_position_to_world;
  goal_pose.orientation = present_orientation_to_world;

  setPose(manipulator_name, tool_name, goal_pose, move_time);
}