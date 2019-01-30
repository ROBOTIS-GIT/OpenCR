/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef ROBOTIS_MANIPULATOR_MANAGER_H_
#define ROBOTIS_MANIPULATOR_MANAGER_H_

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
#else
  #include <eigen3/Eigen/Eigen>
#endif

#include "robotis_manipulator_common.h"

namespace robotis_manipulator
{

class Kinematics
{
public:
  Kinematics() {}
  virtual ~Kinematics() {}

  virtual void setOption(const void *arg) = 0;
  virtual Eigen::MatrixXd jacobian(Manipulator *manipulator, Name tool_name) = 0;
  virtual void solveForwardKinematics(Manipulator *manipulator) = 0;
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_position) = 0;
};

class JointActuator
{
public:
  bool enabled_state_;

  JointActuator() : enabled_state_(false) {}
  virtual ~JointActuator() {}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  virtual std::vector<uint8_t> getId() = 0;

  virtual void enable() = 0;
  virtual void disable() = 0;

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ActuatorValue> value_vector) = 0;
  virtual std::vector<ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id) = 0;

  bool findId(uint8_t actuator_id);
  bool getEnabledState();
};

class ToolActuator
{
public:
  bool enabled_state_;

  ToolActuator():enabled_state_(false){}
  virtual ~ToolActuator() {}

  virtual void init(uint8_t actuator_id, const void *arg) = 0;
  virtual void setMode(const void *arg) = 0;
  virtual uint8_t getId() = 0;

  virtual void enable() = 0;
  virtual void disable() = 0;

  virtual bool sendToolActuatorValue(ActuatorValue value) = 0;
  virtual ActuatorValue receiveToolActuatorValue() = 0;

  bool findId(uint8_t actuator_id);
  bool getEnabledState();
};


class CustomJointTrajectory
{
public:
  CustomJointTrajectory() {}
  virtual ~CustomJointTrajectory() {}

  virtual void makeJointTrajectory(double move_time, JointWaypoint start, const void *arg) = 0; 
  virtual void setOption(const void *arg) = 0;
  virtual JointWaypoint getJointWaypoint(double tick) = 0;
};

class CustomTaskTrajectory
{
public:
  CustomTaskTrajectory() {}
  virtual ~CustomTaskTrajectory() {}

  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg) = 0; 
  virtual void setOption(const void *arg) = 0;
  virtual TaskWaypoint getTaskWaypoint(double tick) = 0;
};

} // namespace ROBOTIS_MANIPULATOR
#endif
