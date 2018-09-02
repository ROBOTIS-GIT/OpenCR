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
                                     processing_(false),
                                     drawing_(false),
                                     draw_cnt_(0)
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

void OpenManipulator::addDraw(Name name, Draw *draw)
{
  Draw *temp_draw = draw;

  draw_.insert(std::make_pair(name, temp_draw));
}

void OpenManipulator::initJointTrajectory()
{
  joint_trajectory_ = new OM_PATH::JointTrajectory(manipulator_.getDOF());

  std::vector<float> empty_v;

  for (uint8_t index = 0; index < manipulator_.getDOF(); index++)
    empty_v.push_back(0.0);

  if (processing_)
    previous_goal_.position = empty_v;

  if (platform_)
    previous_goal_.position = receiveAllActuatorAngle();
  
  previous_goal_.velocity = empty_v;
  previous_goal_.acceleration = empty_v;

  start_trajectory_.reserve(manipulator_.getDOF());
  goal_trajectory_.reserve(manipulator_.getDOF());
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

void OpenManipulator::addWorld(Name world_name,
                               Name child_name,
                               Vector3f world_position,
                               Matrix3f world_orientation)
{
  manipulator_.addWorld(world_name, child_name, world_position, world_orientation);
}

void OpenManipulator::addComponent(Name my_name,
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
  manipulator_.addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation, actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addTool(Name my_name,
                              Name parent_name,
                              Vector3f relative_position,
                              Matrix3f relative_orientation,
                              int8_t tool_id,
                              float coefficient,
                              float mass,
                              Matrix3f inertia_tensor,
                              Vector3f center_of_mass)
{
  manipulator_.addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addComponentChild(Name my_name, Name child_name)
{
  manipulator_.addComponentChild(my_name, child_name);
}

void OpenManipulator::checkManipulatorSetting()
{
  manipulator_.checkManipulatorSetting();
}

void OpenManipulator::setWorldPose(Pose world_pose)
{
  manipulator_.setWorldPose(world_pose);
}

void OpenManipulator::setWorldPosition(Vector3f world_position)
{
  manipulator_.setWorldPosition(world_position);
}

void OpenManipulator::setWorldOrientation(Matrix3f world_orientation)
{
  manipulator_.setWorldOrientation(world_orientation);
}

void OpenManipulator::setWorldState(State world_state)
{
  manipulator_.setWorldState(world_state);
}

void OpenManipulator::setWorldVelocity(VectorXf world_velocity)
{
  manipulator_.setWorldVelocity(world_velocity);
}

void OpenManipulator::setWorldAcceleration(VectorXf world_acceleration)
{
  manipulator_.setWorldAcceleration(world_acceleration);
}

void OpenManipulator::setComponent(Name name, Component component)
{
  manipulator_.setComponent(name, component);
}

void OpenManipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  manipulator_.setComponentPoseToWorld(name, pose_to_world);
}

void OpenManipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world)
{
  manipulator_.setComponentPositionToWorld(name, position_to_world);
}

void OpenManipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd)
{
  manipulator_.setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void OpenManipulator::setComponentStateToWorld(Name name, State state_to_world)
{
  manipulator_.setComponentStateToWorld(name, state_to_world);
}

void OpenManipulator::setComponentVelocityToWorld(Name name, VectorXf velocity)
{
  manipulator_.setComponentVelocityToWorld(name, velocity);
}

void OpenManipulator::setComponentAccelerationToWorld(Name name, VectorXf accelaration)
{
  manipulator_.setComponentAccelerationToWorld(name, accelaration);
}

void OpenManipulator::setComponentJointAngle(Name name, float angle)
{
  manipulator_.setComponentJointAngle(name, angle);
}

void OpenManipulator::setComponentJointVelocity(Name name, float angular_velocity)
{
  manipulator_.setComponentJointVelocity(name, angular_velocity);
}

void OpenManipulator::setComponentJointAcceleration(Name name, float angular_acceleration)
{
  manipulator_.setComponentJointAcceleration(name, angular_acceleration);
}

void OpenManipulator::setComponentToolOnOff(Name name, bool on_off)
{
  manipulator_.setComponentToolOnOff(name, on_off);
}

void OpenManipulator::setComponentToolValue(Name name, float actuator_value)
{
  manipulator_.setComponentToolValue(name, actuator_value);
}

void OpenManipulator::setAllActiveJointAngle(std::vector<float> angle_vector)
{
  manipulator_.setAllActiveJointAngle(angle_vector);
}

int8_t OpenManipulator::getDOF()
{
  return manipulator_.getDOF();
}

int8_t OpenManipulator::getComponentSize()
{
  return manipulator_.getComponentSize();
}

Name OpenManipulator::getWorldName()
{
  return manipulator_.getWorldName();
}

Name OpenManipulator::getWorldChildName()
{
  return manipulator_.getWorldChildName();
}

Pose OpenManipulator::getWorldPose()
{
  return manipulator_.getWorldPose();
}

Vector3f OpenManipulator::getWorldPosition()
{
  return manipulator_.getWorldPosition();
}

Matrix3f OpenManipulator::getWorldOrientation()
{
  return manipulator_.getWorldOrientation();
}

State OpenManipulator::getWorldState()
{
  return manipulator_.getWorldState();
}

VectorXf OpenManipulator::getWorldVelocity()
{
  return manipulator_.getWorldVelocity();
}

VectorXf OpenManipulator::getWorldAcceleration()
{
  return manipulator_.getWorldAcceleration();
}

std::map<Name, Component> OpenManipulator::getAllComponent()
{
  return manipulator_.getAllComponent();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorBegin()
{
  return manipulator_.getIteratorBegin();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorEnd()
{
  return manipulator_.getIteratorEnd();
}

Component OpenManipulator::getComponent(Name name)
{
  return manipulator_.getComponent(name);
}

Name OpenManipulator::getComponentParentName(Name name)
{
  return manipulator_.getComponentParentName(name);
}

std::vector<Name> OpenManipulator::getComponentChildName(Name name)
{
  return manipulator_.getComponentChildName(name);
}

Pose OpenManipulator::getComponentPoseToWorld(Name name)
{
  return manipulator_.getComponentPoseToWorld(name);
}

Vector3f OpenManipulator::getComponentPositionToWorld(Name name)
{
  return manipulator_.getComponentPositionToWorld(name);
}

Matrix3f OpenManipulator::getComponentOrientationToWorld(Name name)
{
  return manipulator_.getComponentOrientationToWorld(name);
}

State OpenManipulator::getComponentStateToWorld(Name name)
{
  return manipulator_.getComponentStateToWorld(name);
}

VectorXf OpenManipulator::getComponentVelocityToWorld(Name name)
{
  return manipulator_.getComponentVelocityToWorld(name);
}

VectorXf OpenManipulator::getComponentAccelerationToWorld(Name name)
{
  return manipulator_.getComponentAccelerationToWorld(name);
}

Pose OpenManipulator::getComponentRelativePoseToParent(Name name)
{
  return manipulator_.getComponentRelativePoseToParent(name);
}

Vector3f OpenManipulator::getComponentRelativePositionToParent(Name name)
{
  return manipulator_.getComponentRelativePositionToParent(name);
}

Matrix3f OpenManipulator::getComponentRelativeOrientationToParent(Name name)
{
  return manipulator_.getComponentRelativeOrientationToParent(name);
}

Joint OpenManipulator::getComponentJoint(Name name)
{
  return manipulator_.getComponentJoint(name);
}

int8_t OpenManipulator::getComponentJointId(Name name)
{
  return manipulator_.getComponentJointId(name);
}

float OpenManipulator::getComponentJointCoefficient(Name name)
{
  return manipulator_.getComponentJointCoefficient(name);
}

Vector3f OpenManipulator::getComponentJointAxis(Name name)
{
  return manipulator_.getComponentJointAxis(name);
}

float OpenManipulator::getComponentJointAngle(Name name)
{
  return manipulator_.getComponentJointAngle(name);
}

float OpenManipulator::getComponentJointVelocity(Name name)
{
  return manipulator_.getComponentJointVelocity(name);
}

float OpenManipulator::getComponentJointAcceleration(Name name)
{
  return manipulator_.getComponentJointAcceleration(name);
}

Tool OpenManipulator::getComponentTool(Name name)
{
  return manipulator_.getComponentTool(name);
}

int8_t OpenManipulator::getComponentToolId(Name name)
{
  return manipulator_.getComponentToolId(name);
}

float OpenManipulator::getComponentToolCoefficient(Name name)
{
  return manipulator_.getComponentToolCoefficient(name);
}

bool OpenManipulator::getComponentToolOnOff(Name name)
{
  return manipulator_.getComponentToolOnOff(name);
}

float OpenManipulator::getComponentToolValue(Name name)
{
  return manipulator_.getComponentToolValue(name);
}

float OpenManipulator::getComponentMass(Name name)
{
  return manipulator_.getComponentMass(name);
}

Matrix3f OpenManipulator::getComponentInertiaTensor(Name name)
{
  return manipulator_.getComponentInertiaTensor(name);
}

Vector3f OpenManipulator::getComponentCenterOfMass(Name name)
{
  return manipulator_.getComponentCenterOfMass(name);
}

std::vector<float> OpenManipulator::getAllJointAngle()
{
  return manipulator_.getAllJointAngle();
}

std::vector<float> OpenManipulator::getAllActiveJointAngle()
{
  return manipulator_.getAllActiveJointAngle();
}

std::vector<uint8_t> OpenManipulator::getAllActiveJointID()
{
  return manipulator_.getAllActiveJointID();
}

// KINEMATICS

MatrixXf OpenManipulator::jacobian(Name tool_name)
{
  return kinematics_->jacobian(&manipulator_, tool_name);
}

void OpenManipulator::forward()
{
  return kinematics_->forward(&manipulator_);
}

void OpenManipulator::forward(Name first_component_name)
{
  return kinematics_->forward(&manipulator_, first_component_name);
}

std::vector<float> OpenManipulator::inverse(Name tool_name, Pose goal_pose)
{
  return kinematics_->inverse(&manipulator_, tool_name, goal_pose);
}

// ACTUATOR
bool OpenManipulator::sendAllActuatorAngle(std::vector<float> radian_vector)
{
  std::vector<float> calc_angle;
  std::map<Name, Component>::iterator it;

  uint8_t index = 0;
  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) != -1)
    {
      calc_angle.push_back(radian_vector.at(index++) * manipulator_.getComponentJointCoefficient(it->first));
    }
  }

  return actuator_->sendAllActuatorAngle(calc_angle);
}

bool OpenManipulator::sendMultipleActuatorAngle(std::vector<uint8_t> active_joint_id, std::vector<float> radian_vector)
{
  std::vector<float> calc_angle;
  std::map<Name, Component>::iterator it;

  for (uint8_t index = 0; index < active_joint_id.size(); index++)
  {
    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      if (active_joint_id.at(index) == manipulator_.getComponentJointId(it->first))
      {
        calc_angle.push_back(radian_vector.at(index) * manipulator_.getComponentJointCoefficient(it->first));
        break;
      }
    }
  }

  return actuator_->sendMultipleActuatorAngle(active_joint_id, calc_angle);
}

bool OpenManipulator::sendActuatorAngle(uint8_t active_joint_id, float radian)
{
  float calc_angle;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) == active_joint_id)
    {
      calc_angle = radian * manipulator_.getComponentJointCoefficient(it->first);
    }
  }

  return actuator_->sendActuatorAngle(active_joint_id, calc_angle);
}

bool OpenManipulator::sendActuatorSignal(uint8_t active_joint_id, bool onoff)
{
  return actuator_->sendActuatorSignal(active_joint_id, onoff);
}

std::vector<float> OpenManipulator::receiveAllActuatorAngle()
{
  std::vector<float> angles = actuator_->receiveAllActuatorAngle();
  std::vector<uint8_t> active_joint_id = manipulator_.getAllActiveJointID();
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
  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) != -1)
    {
      calc_sorted_angle.push_back(sorted_angle_vector.at(index++) / manipulator_.getComponentJointCoefficient(it->first));
    }
  }

  return calc_sorted_angle;
}

void OpenManipulator::actuatorInit(const void *arg)
{
  return actuator_->initActuator(arg);
}

void OpenManipulator::setActuatorControlMode()
{
  actuator_->setActuatorControlMode();
}

void OpenManipulator::actuatorEnable()
{
  return actuator_->Enable();
}

void OpenManipulator::actuatorDisable()
{
  return actuator_->Disable();
}

// DRAW
void OpenManipulator::drawInit(Name name, float drawing_time, const void *arg)
{
  drawing_time_ = drawing_time;

  draw_.at(name)->initDraw(arg);
}

void OpenManipulator::setRadiusForDrawing(Name name, float radius)
{
  draw_.at(name)->setRadius(radius);
}

void OpenManipulator::setStartPositionForDrawing(Name name, Vector3f start_position)
{
  draw_.at(name)->setStartPosition(start_position);
  drawing_ = true;
}

Pose OpenManipulator::getPoseForDrawing(Name name, float tick)
{
  return draw_.at(name)->getPose(tick);
}

void OpenManipulator::draw(Name object)
{
  object_ = object;

  drawing_ = true;
  draw_cnt_ = 0;
}

bool OpenManipulator::drawing()
{
  return drawing_;
}

// JOINT TRAJECTORY

void OpenManipulator::setMoveTime(float move_time)
{
  move_time_ = move_time;
}
void OpenManipulator::setPresentTime(float present_time)
{
  present_time_ = present_time;
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
  return joint_trajectory_->getCoefficient();
}

void OpenManipulator::move()
{
  moving_ = true;
  step_cnt_ = 0;
  start_time_ = present_time_;
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

void OpenManipulator::jointControlForDrawing(Name tool_name, bool use_time)
{
  uint16_t step_time = uint16_t(floor(drawing_time_ / control_time_) + 1.0);     //for use step cnt

  float tick_time = 0;

  std::vector<float> goal_position;
  std::vector<float> goal_velocity;
  std::vector<float> goal_acceleration;

  goal_position.reserve(manipulator_.getDOF());
  goal_velocity.reserve(manipulator_.getDOF());
  goal_acceleration.reserve(manipulator_.getDOF());

  if (use_time == false)
  {
    if (drawing_)
    {
      if (draw_cnt_ < step_time)
      {
        tick_time = control_time_ * draw_cnt_;

        if (object_ == LINE)
          goal_position = kinematics_->inverse(&manipulator_, tool_name, line_.getPose(tick_time));
        else
          goal_position = kinematics_->inverse(&manipulator_, tool_name, getPoseForDrawing(object_, tick_time));

        if (platform_)
          sendMultipleActuatorAngle(manipulator_.getAllActiveJointID(), goal_position);

        if (processing_)
        {
          if (platform_ == false)
            manipulator_.setAllActiveJointAngle(goal_position);
          sendAngleToProcessing(goal_position);
        }

        previous_goal_.position = goal_position;

        draw_cnt_++;
      }
      else
      {
        draw_cnt_ = 0;
        drawing_ = false;
      }
    }
  }
}

void OpenManipulator::jointControl(bool use_time)
{
  uint16_t step_time = uint16_t(floor(move_time_ / control_time_) + 1.0);     //for use step cnt
  
  float tick_time = 0;

  std::vector<float> goal_position;
  std::vector<float> goal_velocity;
  std::vector<float> goal_acceleration;

  goal_position.reserve(manipulator_.getDOF());
  goal_velocity.reserve(manipulator_.getDOF());
  goal_acceleration.reserve(manipulator_.getDOF());


  if(use_time == false)          //use step cnt
  {
    ////////////////////////////////////////////////////////
    if (moving_)
    {
      if (step_cnt_ < step_time)
      {
        tick_time = control_time_ * step_cnt_;

        goal_position = joint_trajectory_->getPosition(tick_time);
        goal_velocity = joint_trajectory_->getVelocity(tick_time);
        goal_acceleration = joint_trajectory_->getAcceleration(tick_time);

        if (platform_)
          sendMultipleActuatorAngle(manipulator_.getAllActiveJointID(), goal_position);

        if (processing_)
        {
          if (platform_ == false)
            manipulator_.setAllActiveJointAngle(goal_position);
          sendAngleToProcessing(goal_position);
        }

        previous_goal_.position = goal_position;
        previous_goal_.velocity = goal_velocity;
        previous_goal_.acceleration = goal_acceleration;

        step_cnt_++;
      }
      else
      {
        step_cnt_ = 0;
        moving_ = false;
      }
    }
    /////////////////////////////////////////////////////////
  }
  else                        //use time
  {
    /////////////////////////////////////////////////////////
    if(moving_)
    {
      tick_time = present_time_ - start_time_;
      if(tick_time < move_time_)
      {
        goal_position = joint_trajectory_->getPosition(tick_time);
        goal_velocity = joint_trajectory_->getVelocity(tick_time);
        goal_acceleration = joint_trajectory_->getAcceleration(tick_time);  

        if (platform_)
          sendMultipleActuatorAngle(manipulator_.getAllActiveJointID(), goal_position);

        if (processing_)
        {
          if (platform_ == false)
            manipulator_.setAllActiveJointAngle(goal_position);
          sendAngleToProcessing(goal_position);
        }

        previous_goal_.position = goal_position;
        previous_goal_.velocity = goal_velocity;
        previous_goal_.acceleration = goal_acceleration;

        moving_  = true; 
      }
      else
      {
        goal_position = joint_trajectory_->getPosition(move_time_);
        goal_velocity = joint_trajectory_->getVelocity(move_time_);
        goal_acceleration = joint_trajectory_->getAcceleration(move_time_);  

        if (platform_)
          sendMultipleActuatorAngle(manipulator_.getAllActiveJointID(), goal_position);

        if (processing_)
        {
          if (platform_ == false)
            manipulator_.setAllActiveJointAngle(goal_position);
          sendAngleToProcessing(goal_position);
        }
        moving_   = false; 
        start_time_ = present_time_;
      }
    }
    else
    {
      start_time_ = present_time_;
    }
    /////////////////////////////////////////////////////////
  }
}

void OpenManipulator::jointMove(std::vector<float> goal_position, float move_time)
{
  Trajectory start;
  Trajectory goal;

  std::vector<float> present_position = previous_goal_.position;
  std::vector<float> present_velocity = previous_goal_.velocity;
  std::vector<float> present_acceleration = previous_goal_.acceleration;

  start_trajectory_.clear();
  goal_trajectory_.clear();

  for (uint8_t index = 0; index < manipulator_.getDOF(); index++)
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
  makeTrajectory(start_trajectory_, goal_trajectory_);
  move();
}

bool OpenManipulator::toolMove(Name tool_name, bool onoff)
{
  manipulator_.setComponentToolOnOff(tool_name, onoff);

  if (platform_)
    actuator_->sendActuatorSignal(manipulator_.getComponentToolId(tool_name), onoff);

  if (processing_)
    OM_PROCESSING::sendToolData2Processing(onoff);

  return true;
}

bool OpenManipulator::toolMove(Name tool_name, float tool_value)
{
  float calc_value = tool_value * manipulator_.getComponentToolCoefficient(tool_name);

  manipulator_.setComponentToolValue(tool_name, calc_value);

  if (platform_)
    actuator_->sendActuatorAngle(manipulator_.getComponentToolId(tool_name), calc_value);

  if (processing_)
    OM_PROCESSING::sendToolData2Processing(tool_value);

  return true;
}

void OpenManipulator::setPose(Name tool_name, Pose goal_pose, float move_time)
{
  std::vector<float> goal_position = kinematics_->inverse(&manipulator_, tool_name, goal_pose);

  // DEBUG.print("Inverse result : ");
  // DEBUG.print(goal_position.size());
  // DEBUG.print(" 1: ");
  // DEBUG.print(goal_position.at(0));
  // DEBUG.print(", 2: ");
  // DEBUG.print(goal_position.at(1));
  // DEBUG.print(", 3: ");
  // DEBUG.print(goal_position.at(2));
  // DEBUG.println();

  jointMove(goal_position, move_time);
}

void OpenManipulator::setMove(Name tool_name, Vector3f meter, float move_time)
{
  setAllActiveJointAngle(previous_goal_.position);
  forward(manipulator_.getWorldChildName());

  previous_goal_.pose = manipulator_.getComponentPoseToWorld(tool_name);

  if (platform_)
  {
    setAllActiveJointAngle(receiveAllActuatorAngle());
    forward(manipulator_.getWorldChildName());
  }

  Vector3f present_position_to_world = previous_goal_.pose.position;
  Matrix3f present_orientation_to_world = previous_goal_.pose.orientation;

  Vector3f goal_position_to_world = present_position_to_world + meter;

  Pose goal_pose;
  goal_pose.position = goal_position_to_world;
  goal_pose.orientation = present_orientation_to_world;

  // DEBUG.println("-----------------------------------------------");
  // DEBUG.print("GoalPos result : ");
  // DEBUG.print(goal_pose.position.size());
  // DEBUG.print(" x: ");
  // DEBUG.print(goal_pose.position(0));
  // DEBUG.print(", y: ");
  // DEBUG.print(goal_pose.position(1));
  // DEBUG.print(", z: ");
  // DEBUG.print(goal_pose.position(2));
  // DEBUG.println();

  setPose(tool_name, goal_pose, move_time);
}

void OpenManipulator::drawLine(Vector3f start, Vector3f end, float move_time)
{
  drawing_time_ = move_time;
  
  line_.init(move_time, control_time_);
  line_.setTwoPoints(start, end);

  draw(LINE);
}