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
   
#ifndef OMAPI_HPP_
#define OMAPI_HPP_

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
void connectSetGetAngle(float* (*fp)()){ ACTUATOR::getAngle = fp; }

namespace KINEMATICS
{
void (*foward)(void) = NULL;
void (*inverse)(void) = NULL;
} // namespace KINEMATICS

void connectForward(void (*fp)(void)){KINEMATICS::foward = fp;}
void connectInverse(void (*fp)(void)){KINEMATICS::inverse = fp;}

namespace PATH
{
void (*line)(void) = NULL;
void (*arc)(void) = NULL;
void (*custom)(void) = NULL;
} // namespace PATH

void connectLine(void (*fp)(void)){PATH::line = fp;}
void connectArc(void (*fp)(void)){PATH::arc = fp;}
void connectCustom(void (*fp)(void)){PATH::custom = fp;}

// Basic function

/////////////////////////////////////////////////////////////

//init fuction
void initManipulator(Manipulator* manipulator, int8_t dof, int8_t joint_size, int8_t link_size, int8_t tool_size, bool* error = false)
{
  manipulator.initManipulator(dof, joint_size, link_size, tool_size, error);
}

void makeBase(Manipulator* manipulator, char* base_name, Eigen::Vector3f base_position = Eigen::Vector3f::Zero(), Eigen::Matrix3f base_orientation = Eigen::Matrix3f::Identity(3,3), bool* error = false)
{
  manipulator.makeBase(base_name, base_position, base_orientation, error);
}

void makeJoint(Manipulator* manipulator, char* joint_name, int8_t actuator_id = -1, Eigen::Vector3f axis = Eigen::Vector3f::Zero(), bool* error = false)
{
  manipulator.makeJoint(joint_name, actuator_id, axis, error);
}

void makeTool(Manipulator* manipulator, char* tool_name, int8_t actuator_id, Eigen::Vector3f axis, bool* error = false)
{
  manipulator.makeTool(tool_name, actuator_id, axis, error);
}

void makeLink(Manipulator* manipulator, char* link_name, int8_t inner_control_point_size, bool* error = false)
{
  manipulator.makeLink(link_name, inner_control_point_size, error);
}

void addControlPointInLink(Manipulator* manipulator, char* link_name, char* point_name, Eigen::Vector3f relative_position = Eigen::Vector3f::Zero(), Eigen::Matrix3f relative_orientation = Eigen::Matrix3f::Identity(3,3), bool* error = false)
{
  manipulator.addControlPoint(link_name, point_name, relative_position, relative_orientation, error);
}

void setLinkCenterOfMass(Manipulator* manipulator, char* link_name, float mass, Eigen::Matrix3f initial_inertia_tensor, Eigen::Vector3f relative_position = Eigen::Vector3f::Zero(), Eigen::Matrix3f relative_orientation = Eigen::Matrix3f::Identity(3,3), bool* error = false)
{
  manipulator.setCenterOfMass(link_name, mass, initial_inertia_tensor, relative_position, relative_orientation, error);
}

void checkManipulatorSetting(Manipulator* manipulator, bool* error = false)
{
  //check if all control point object is include in some link.
  //check if all link has more than one control point.
  //check if all map.size is same to int parameter
}

//set fuction

  //pose set
void setPose(Manipulator* manipulator, char* control_point_name, Pose pose, bool* error = false)
{
  manipulator.setPose(control_point_name, pose, error);
}

void setPosition(Manipulator* manipulator, char* control_point_name, Eigen::Vector3f position, bool* error = false)
{
  manipulator.setPostion(control_point_name, position, error);
}

void setOrientation(Manipulator* manipulator, char* control_point_name, Eigen::Matrix3f orientation, bool* error = false)
{
  manipulator.setOrientation(control_point_name, orientation, error);
}

void setDynamicPose(Manipulator* manipulator, char* control_point_name, DynamicPose dynamic_pose, bool* error = false)
{
  manipulator.setDynamicPose(control_point_name, dynamic_pose, error);
}

void setLinearVelocity(Manipulator* manipulator, char* control_point_name, Eigen::Vector3f linear_velocity, bool* error = false)
{
  manipulator.setLinearVelocity(control_point_name, linear_velocity, error);
}

void setAngularVelocity(Manipulator* manipulator, char* control_point_name, Eigen::Vector3f angular_velocity, bool* error = false)
{
  manipulator.setAngularVelocity(control_point_name, angular_velocity, error);
}

void setLinearAcceleration(Manipulator* manipulator, char* control_point_name, Eigen::Vector3f linear_acceleration, bool* error = false)
{
  manipulator.setLinearAcceleration(control_point_name, linear_acceleration, error);
}

void setAngularAcceleration(Manipulator* manipulator, char* control_point_name, Eigen::Vector3f angular_acceleration, bool* error = false)
{
  manipulator.setAngularAcceleration(control_point_name, angular_acceleration, error);
}

  //joint set
void setJointState(Manipulator* manipulator, char* joint_name, JointState state, bool* error = false)
{
  manipulator.setJointState(joint_name, state, error);
}

void setJointAngle(Manipulator* manipulator, char* joint_name, float angle, bool* error = false)
{
  manipulator.setJointAngle(joint_name, angle, error);
}

void setJointAngularVelocity(Manipulator* manipulator, char* joint_name, float velocity, bool* error = false)
{
  manipulator.setJointAngularVelocity(joint_name, velocity, error);
}

void setJointAngularAcceleration(Manipulator* manipulator, char* joint_name, float acceleration, bool* error = false)
{
  manipulator.setJointAngularAcceleration(joint_name, acceleration, error);
}

  //tool set
void setToolOnOff(Manipulator* manipulator, char* tool_name, bool on_off, bool* error = false)
{
  manipulator.setToolOnOff(tool_name, on_off, error);
}

void setToolActuatorValue(Manipulator* manipulator, char* tool_name, float actuator_value, bool* error = false)
{
  manipulator.setToolActuatorValue(tool_name, actuator_value, error);
}

//Get function
  //manipulator get
int8_t getManipulatorDOF(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getDOF(error);
}

int8_t getManipulatorJointSize(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getJointSize(error);
}

int8_t getManipulatorLinkSize(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getLinkSize(error);;
}

int8_t getManipulatorToolSize(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getToolSize(error);;
}

int8_t getManipulatorControlPointSize(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getControlPointSize(error);;
}

char* getBaseName(Manipulator* manipulator, bool* error = false)
{
  return manipulator.getBaseName();
}

  //link parameter get
RelativePose getLinkParameter(Manipulator* manipulator, char* link_name, char* point_name, bool* error = false)
{
  return manipulator.getRelativePose(link_name, point_name, error);
}

RelativePose getLinkParameter(Manipulator* manipulator, char* link_name, char* to_point_name, char* from_point_name, bool* error = false)
{
  return manipulator.getRelativePose(link_name, to_point_name, from_point_name, error);
}

RelativePose getRelativePose(Manipulator* manipulator, char* link_name, char* point_name, bool* error = false)
{
  return manipulator.getRelativePose(link_name, point_name, error);
}

RelativePose getRelativePose(Manipulator* manipulator, char* link_name, char* to_point_name, char* from_point_name, bool* error = false)
{
  return manipulator.getRelativePose(link_name, to_point_name, from_point_name, error);
}

float getLinkMass(Manipulator* manipulator, char* link_name, bool* error = false)
{
  return manipulator.getMass(link_name, error);
}

Eigen::Matrix3f getLinkInitialInertiaTensor(Manipulator* manipulator, char* link_name, bool* error = false)
{
  return manipulator.getInitialInertiaTensor(link_name, error);
}

RelativePose getRelativeCenterOfMassPose(Manipulator* manipulator, char* link_name, bool* error = false)
{
  return manipulator.getRelativeCenterOfMassPose(link_name, error);
}

RelativePose getRelativeCenterOfMassPose(Manipulator* manipulator, char* link_name, char* from_name, bool* error = false)
{
  return manipulator.getRelativeCenterOfMassPose(link_name, from_name, error);
}

  //control point get
Eigen::Vector3f getPosition(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getPosition(point_name, error);
}

Eigen::Matrix3f getOrientation(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getOrientation(point_name, error);
}

Pose getPose(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getPose(point_name, error);
}

Eigen::Vector3f getLinearVelocity(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getLinearVelocity(point_name, error);
}

Eigen::Vector3f getAngularVelocity(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getAngularVelocity(point_name, error);
}

Eigen::Vector3f getLinearAcceleration(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getLinearAcceleration(point_name, error);
}
  
Eigen::Vector3f getAngularAcceleration(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getAngularAcceleration(point_name, error);
}

DynamicPose getDynamicPose(Manipulator* manipulator, char* point_name, bool* error = false)
{
  return manipulator.getDynamicPose(point_name, error);
}

  //joint parameter get
int8_t getJointActuatorId(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointActuatorId(joint_name, error);
}

Eigen::Vector3f getJointAxis(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointAxis(joint_name, error);
}

float getJointAngle(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointAngle(joint_name, error);
}

float getJointAngularVelocity(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointAngularVelocity(joint_name, error);
}

float getJointAngularAcceleration(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointAngularAcceleration(joint_name, error);
}

JointState getJointJointState(Manipulator* manipulator, char* joint_name, bool* error = false)
{
  return manipulator.getJointJointState(joint_name, error);
}

  //tool parameter get
int8_t getToolActuatorId(Manipulator* manipulator, char* tool_name, bool* error = false)
{
  return manipulator.getToolActuatorId(tool_name, error);
}

Eigen::Vector3f getToolAxis(Manipulator* manipulator, char* tool_name, bool* error = false)
{
  return manipulator.getToolAxis(tool_name, error);
}

bool getToolOnOff(Manipulator* manipulator, char* tool_name, bool* error = false)
{
  return manipulator.getToolOnOff(tool_name, error);
}

float getToolActuatorValue(Manipulator* manipulator, char* tool_name, bool* error = false)
{
  return manipulator.getToolActuatorValue(tool_name, error);
}

/////////////////////////////////////////////////////////////





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
   
   
   
   
   
   
  