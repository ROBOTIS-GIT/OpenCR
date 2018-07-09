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

/* Authors: Hye-Jong KIM*/


#include "../../include/open_manipulator/OMManager.h"

Eigen::Vector3f MakeEigenVector3(float v1, float v2, float v3)
{
  Eigen::Vector3f temp;
  temp << v1, v2, v3;
  return temp;
}

Eigen::Matrix3f MakeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
{
  Eigen::Matrix3f temp;
  temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return temp;
}

Eigen::Matrix3f makeRotationMatrix(float roll, float pitch, float yaw)
{
  Eigen::Matrix3f rotation_matrix;
  Eigen::Matrix3f roll_matrix;
  Eigen::Matrix3f pitch_matrix;
  Eigen::Matrix3f yaw_matrix;

  roll_matrix  << 1.000,  0.000,     0.000,
                  0.000,  cos(roll), sin(roll),
                  0.000, -sin(roll), cos(roll);

  pitch_matrix << cos(pitch),  0.000, sin(pitch),
                  0.000,       1.000, 0.000,
                  -sin(pitch), 0.000, cos(pitch);

  yaw_matrix   << cos(yaw), -sin(yaw), 0.000,
                  sin(yaw), cos(yaw),  0.000,
                  0.000,    0.000,     1.000;
  
  rotation_matrix = roll_matrix * pitch_matrix * yaw_matrix;

  return rotation_matrix;
}

//////////////////////////////////////Manipulator////////////////////////////////////////////
Manipulator::Manipulator(int8_t dof):
name("UnknownManipulator")
{
  dof_ = dof;
}

Manipulator::~Manipulator(){}

void Manipulator::setDOF(int8_t dof)
{
  dof_=dof;
}

//////////////////////////////////////Joint////////////////////////////////////////////

Joint::Joint(): 
dxl_id_(-1),
{
  joint_state_.angle = 0.0;
  joint_state_.angular_velocity = 0.0;
  joint_state_.angular_acceleration = 0.0;
  joint_pose_.position = Eigen::Vector3f::Zero();
  joint_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
  axis_ = MakeEigenVector3(0,0,0);
}

Joint::~Joint(){}

void Joint::init(int8_t dxl_id, Eigen::Vector3f axis)
{
  dxl_id_ = dxl_id;
  axis_ = axis;
}

int8_t Joint::getId()
{
  return dxl_id_;
}

Eigen::Vector3f getAxis()
{
  return axis_;
}

void Joint::setAngle(float angle)
{
  joint_state_.angle = angle;
}

void Joint::setAngularVelocity(float angular_velocity)
{
  joint_state_.angular_velocity = angular_velocity;
}

void Joint::setAngularAcceleration(float angular_acceleration)
{
  joint_state_.angular_acceleration = angular_acceleration;
}

void setJointState(State joint_state)
{
  joint_state_.angle = joint_state.angle;
  joint_state_.angular_velocity = joint_state.angular_velocity;
  joint_state_.angular_acceleration = joint_state.angular_acceleration;
}

float Joint::getAngle()
{
  return joint_state_.angle_;
}

float Joint::getAngularVelocity()
{
  return joint_state_.angular_velocity;
}
    
float Joint::getAngularAcceleration()
{
  return joint_state_.angular_acceleration;
}

State Joint::getJointState()
{
  State temp; 
  temp.angle = joint_state_.angle;
  temp.angular_velocity = joint_state_.angular_velocity;
  temp.angular_acceleration = joint_state_.angular_acceleration;
  return temp;
}

void Joint::setPosition(Eigen::Vector3f position)
{
  joint_pose_.position = position;
}

void Joint::setOrientation(Eigen::Matrix3f orientation)
{
  joint_pose_.orientation = orientation;
}

void Joint::setPose(Pose joint_pose)
{
  joint_pose_.position = joint_pose.position;
  joint_pose_.orientation = joint_pose.orientation;
}

Eigen::Vector3f Joint::getPosition()
{
  return joint_pose_.position;
}

Eigen::Matrix3f Joint::getOrientation()
{
  return joint_pose_.orientation;
}

Pose Joint::getPose()
{
  Pose temp;
  temp.position = joint_pose_.position;
  temp.orientation = joint_pose_.orientation;
  return temp;
}

//////////////////////////////////////Link////////////////////////////////////////////

Link::Link():
inner_joint_size_(1),
{
  inertia_.mass = 0;
  inertia_.center_position = Eigen::Vector3f::Zero();
  inertia_.moment = 0;
}

Link::~Link(){}

void Link::init(int8_t inner_joint_size)
{
  inner_joint_size_ = inner_joint_size;
  inner_joint_.resize(inner_joint_size_);
}

void Link::setInertia(Inertia inertia)
{
  inertia_ = inertia;
}

void Link::setMass(float mass)
{
  inertia_.mass = mass;
}

void Link::setCenterPosition(Eigen::Vector3f center_position)
{
  inertia_.center_position =center_position;
}

void Link::setInertiaMoment(float inertia_moment)
{
  inertia_.moment = inertia_moment;
}

Inertia Link::getInertia()
{
  return inertia_;
}

float Link::getMass()
{
  return inertia_.mass;
}

Eigen::Vector3f Link::getRelativeCenterPosition()
{
  return inertia_.center_position;
}

Eigen::Vector3f Link::getRelativeCenterPosition(int8_t from)
{
  Eigen::Vector3f temp;
  temp = inertia_.center_position - inner_joint_.at(FineJoint(from)).relative_position;
  return temp;
}

float Link::getInertiaMoment()
{
  return inertia_.moment;
}

void Link::setCenterPosition(Eigen::Vector3f center_position)
{
  center_position_ = center_position;
}

Eigen::Vector3f Link::getCenterPosition()
{
  return center_position_;
}


int8_t Link::getInnerJointSize()
{
  return inner_joint_size_;
}

void Link::setInnerJoint(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
{
  int8_t i;
  if(findJoint(joint_number) >= 0)
  {
    inner_joint_.at(findJoint(joint_number)).joint_number = joint_number;
    inner_joint_.at(findJoint(joint_number)).relative_position = relative_position;
    inner_joint_.at(findJoint(joint_number)).relative_orientation = relative_orientation;
    return 0;
  }
  else
  {
    for(i=0; i>inner_joint_size_;i++)
    {
      if(findJoint(i) == -1)
      {
        inner_joint_.at(findJoint(i)).joint_number = joint_number;
        inner_joint_.at(findJoint(i)).relative_position = relative_position;
        inner_joint_.at(findJoint(i)).relative_orientation = relative_orientation;
        return 0;
      }
    }
  }
}

InnerJoint Link::getInnerJointInformation(int8_t joint_number)
{
  InnerJoint temp;
  temp.joint_number = inner_joint_.at(findJoint(joint_number)).joint_number;
  temp.relative_position = inner_joint_.at(findJoint(joint_number)).relative_position;
  temp.relative_orientation = inner_joint_.at(findJoint(joint_number)).relative_orientation;
  return temp;
}

Eigen::Vector3f Link::getRelativeJointPosition(int8_t to, int8_t from)
{
  Eigen::Vector3f temp;
  temp = inner_joint_.at(FindJoint(to)).relative_position - inner_joint_.at(FindJoint(from)).relative_position;
  return temp; 
}

Eigen::Matrix3f Link::getRelativeJointOrientation(int8_t to, int8_t from)
{
  Eigen::Matrix3f temp;
  temp = inner_joint_.at(FindJoint(from)).relative_orientation.transpose() * inner_joint_.at(FindJoint(to)).relative_orientation;
  return temp; 
}

Eigen::Vector3f Link::getRelativeJointPosition(int8_t joint_number)
{
  Eigen::Vector3f temp;
  temp = inner_joint_.at(FindJoint(joint_number)).relative_position;
  return temp; 
}

Eigen::Matrix3f Link::getRelativeJointOrientation(int8_t joint_number)
{
  Eigen::Matrix3f temp;
  temp = inner_joint_.at(FindJoint(joint_number)).relative_orientation;
  return temp; 
}

int8_t Link::findJoint(int8_t joint_number)
{
  int8_t i;
  for(i=0; i < inner_joint_size_; i++)
  {
    if(inner_joint_.at(i).joint_number == joint_number)
    {
      return i;
    }
  }
  return -1;
}

//////////////////////////////////////////Base///////////////////////////////////////////////

Base::Base(): 
{
  base_pose_.position = Eigen::Vector3f::Zero();
  base_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
  relative_base_pose_.position = Eigen::Vector3f::Zero();
  relative_base_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
}
Base::~Base(){}

void Base::setRelativeBasePosition(Eigen::Vector3f relative_base_position)
{
  relative_base_pose_.position = relative_base_position;
}

void Base::setRelativeBaseOrientation(Eigen::Matrix3f relative_base_orientation)
{
  relative_base_pose_.orientation = relative_base_orientation;
}

void Base::setRelativeBasePose(Pose relative_base_pose)
{
  relative_base_pose_.position = relative_base_pose.position;
  relative_base_pose_.orientation = relative_base_pose.orientation;
}
    
void Base::setPosition(Eigen::Vector3f base_position)
{
  base_pose_.position = base_position;
}

void Base::setOrientation(Eigen::Matrix3f base_orientation)
{
  base_pose_.orientation = base_orientation;
}

void Base::setPose(Pose base_pose)
{
  base_pose_.position = base_pose.position;
  base_pose_.orientation = base_pose.orientation;
}

Eigen::Vector3f Base::getPosition()
{
  return base_pose_.position;
}

Eigen::Matrix3f Base::getOrientation()
{
  return base_pose_.orientation;
}

Pose Base::getPose()
{
  Pose temp;
  temp.position = base_pose_.position;
  temp.orientation = base_pose_.orientation;
   return temp;
}

Eigen::Vector3f Link::getRelativeBaseJointPosition(int8_t to)
{
  Eigen::Vector3f temp;
  temp = getRelativeJointPosition(to) - relative_base_pose_.position;
  return temp; 
}

Eigen::Matrix3f Link::getRelativeBaseJointOrientation(int8_t to)
{
  Eigen::Matrix3f temp;
  temp = relative_base_pose_.orientation.transpose() * getRelativeJointorientation(to);
  return temp; 
}


//////////////////////////////////////Tool////////////////////////////////////////////

Tool::Tool():
tool_type_(0),
{
  tool_pose_.position = Eigen::Vector3f::Zero();
  tool_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
  relatvie_tool_pose_.position = Eigen::Vector3f::Zero();
  relatvie_tool_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
}

Tool::~Tool(){}

void Tool::setToolType(int8_t tool_type)
{
  tool_type_ = tool_type;
}

int8_t Tool::getToolType()
{
  return tool_type_;
}

void Tool::setRelativeToolPosition(Eigen::Vector3f relatvie_tool_position)
{
  relatvie_tool_pose_.position = relatvie_tool_position;
}

void Tool::setRelativeToolOrientation(Eigen::Matrix3f relatvie_tool_orientation)
{
  relatvie_tool_pose_.orientation = relatvie_tool_orientation;
}

void Tool::setRelativeToolPose(Pose tool_pose)
{
  relatvie_tool_pose_.position = relatvie_tool_pose.position;
  relatvie_tool_pose_.orientation = relatvie_tool_pose.orientation;
}

void Tool::setPosition(Eigen::Vector3f tool_position)
{
  tool_pose_.position = tool_position;
}

void Tool::setOrientation(Eigen::Matrix3f tool_orientation)
{
  tool_pose_.orientation = tool_orientation;
}

void Tool::setPose(Pose tool_pose)
{
  tool_pose_.position = tool_pose.position;
  tool_pose_.orientation = tool_pose.orientation;
}

Eigen::Vector3f Tool::getPosition()
{
  return tool_pose_.position;
}

Eigen::Matrix3f Tool::getOrientation()
{
  return tool_pose_.orientation;
}

Pose Tool::getPose()
{
  Pose temp;
  temp.position = tool_pose_.position;
  temp.orientation = tool_pose_.orientation;
   return temp;
}

Eigen::Vector3f Link::getRelativeToolPosition(int8_t from)
{
  Eigen::Vector3f temp;
  temp = relative_tool_pose_.position - getRelativeJointPosition(from);
  return temp; 
}

Eigen::Matrix3f Link::getRelativeToolOrientation(int8_t from)
{
  Eigen::Matrix3f temp;
  temp = getRelativeJointorientation(from).transpose() * relative_tool_pose_.orientation;
  return temp; 
}


