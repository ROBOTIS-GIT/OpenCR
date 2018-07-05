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

Eigen::Vector3f Link::getCenterPosition()
{
  return inertia_.center_position;
}

Eigen::Vector3f Link::getCenterPosition(int8_t from)
{
  Eigen::Vector3f temp;
  temp = inertia_.center_position - inner_joint_.at(FineJoint(from)).relative_position;
  return temp;
}

float Link::getInertiaMoment()
{
  return inertia_.moment;
}

int8_t getInnerJointSize()
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
        return
      }
      else
      {
        //error
      }
    }
  }
}


JointInLink Link::getJointInformation(int8_t joint_in_link_number)
{
  JointInLink temp;
  temp.number = jointinlink_.at(joint_in_link_number).number;
  temp.relative_position = jointinlink_.at(joint_in_link_number).relative_position;
  temp.relative_orientation = jointinlink_.at(joint_in_link_number).relative_orientation;
  return temp;
}

Eigen::Vector3f Link::getRelativeJointPosition(int8_t to, int8_t from)
{
  Eigen::Vector3f temp;
  temp = jointinlink_.at(FindJoint(to)).relative_position - jointinlink_.at(FindJoint(from)).relative_position;
  return temp; 
}

Eigen::Vector3f Link::getRelativeJointOrientation(int8_t to, int8_t from)
{
  Eigen::Vector3f temp;
  temp = jointinlink_.at(FindJoint(from)).relative_orientation.transpose() * jointinlink_.at(FindJoint(to)).relative_orientation;
  return temp; 
}



int8_t Link::findJoint(int8_t joint_number)
{
  int8_t i;
  for(i=0; i < number_of_joint_in_link_; i++)
  {
    if(jointinlink_.at(i).number == joint_number)
    {
      return i;
    }
  }
  return -1;
}

    







//////////////////////////////////////////Base///////////////////////////////////////////////

Base::Base(): 
counter_(0),
number_of_base_joint_(0),
mass_(0.0),
inertia_moment_(0.0)
{
  center_position_ = Eigen::Vector3f::Zero();
  base_position_ = Eigen::Vector3f::Zero();
  base_orientation_ = Eigen::Matrix3f::Identity(3,3);
}
Base::~Base(){}

void Base::init(int8_t number_of_base_joint)
{
  number_of_base_joint_ = number_of_base_joint;
  base_joint_.resize(number_of_base_joint_);
}

void Base::init(int8_t number_of_base_joint, float mass, Eigen::Vector3f center_position)
{
  number_of_base_joint_ = number_of_base_joint;
  base_joint_.resize(number_of_base_joint_);
  mass_ = mass;
  center_position_ = center_position;
}

void Base::init(int8_t number_of_base_joint, float mass, float inertia_moment, Eigen::Vector3f center_position)
{
  number_of_base_joint_ = number_of_joint_in_link;
  base_joint_.resize(number_of_base_joint_);
  inertia_moment_ = inertia_moment;
  mass_ = mass;
  center_position_ = center_position;
}

void Base::setBaseJoint(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
{
  base_joint_.at(counter_).number = number;
  base_joint_.at(counter_).relative_position = relative_position; 
  base_joint_.at(counter_).relative_orientation = relative_orientation;
  if(counter_ >= number_of_base_joint_){counter_++;}
}

void Base::setBaseJoint(int8_t base_joint_number, int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
{
  base_joint_.at(base_joint_number).number = number;
  base_joint_.at(base_joint_number).relative_position = relative_position; 
  base_joint_.at(base_joint_number).relative_orientation = relative_orientation;
}

JointInLink Base::getJointInformation(int8_t base_joint_number)
{
  JointInLink temp;
  temp.number = base_joint_.at(base_joint_number).number;
  temp.relative_position = base_joint_.at(base_joint_number).relative_position;
  temp.relative_orientation = base_joint_.at(base_joint_number).relative_orientation;
  return temp;
}

int8_t Base::getTheNumberOfJoint()
{
  return number_of_base_joint_;
}

float Base::getMass()
{
  return mass_;
}

void Base::setMass(float mass)
{
  mass_ = mass;
}

float Base::getInertiaMoment()
{
  return inertia_moment_;
}

void Base::setInertiaMoment(float inertia_moment)
{
  inertia_moment_ = inertia_moment;
}

int8_t Base::findJoint(int8_t joint_number)
{
  int8_t i;
  for(i=0; i < number_of_base_joint_; i++)
  {
    if(base_joint_.at(i).number == joint_number)
    {
      return i;
    }
  }
}

Eigen::Vector3f Base::getCenterPosition()
{
  return center_position_;
}
    
Eigen::Vector3f Base::getBaseJointPosition(int8_t to)
{
  Eigen::Vector3f temp;
  temp = base_joint_.at(FindJoint(to)).relative_position - base_position_;
  return temp; 
}

Eigen::Vector3f Base::getBaseJointOrientation(int8_t to)
{
  Eigen::Vector3f temp;
  temp = base_orientation_.transpose() * base_joint_.at(FindJoint(to)).relative_orientation;
  return temp; 
}

void Base::setBasePosition(Eigen::Vector3f base_position)
{
  base_position_ = base_position;
}

void Base::setBaseOrientation(Eigen::Matrix3f base_orientation)
{
  base_orientation_ = base_orientation;
}

void Base::setBasePose(Pose base_pose)
{
  base_position_ = base_pose.position;
  base_orientation_ = base_pose.orientation;
}

Eigen::Vector3f Base::getBasePosition()
{
  return base_position_;
}

Eigen::Matrix3f Base::getBaseOrientation()
{
  return base_orientation_;
}

Pose Base::getBasePose()
{
  Pose temp;
  temp.position = base_position_;
  temp.orientation = base_orientation_;
   return temp
}

//////////////////////////////////////Joint////////////////////////////////////////////

Joint::Joint(): 
dxl_id_(-1),
angle_(0.0),
velocity_(0.0),
acceleration_(0.0)
{
  position_ = Eigen::Vector3f::Zero();
  orientation_ = Eigen::Matrix3f::Identity(3,3);
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
  angle_ = angle;
}
    
float Joint::getAngle()
{
  return angle_;
}

void Joint::setVelocity(float velocity)
{
  velocity_ = velocity;
}

float Joint::getVelocity()
{
  return velocity_;
}
    
void Joint::setAcceleration(float acceleration)
{
  acceleration_ = acceleration;
}

float Joint::getAcceleration()
{
  return acceleration_;
}

void Joint::setPosition(Eigen::Vector3f position)
{
  position_ = position;
}

Eigen::Vector3f Joint::getPosition()
{
  return position_;
}

void Joint::setOrientation(Eigen::Matrix3f orientation)
{
  orientation_ = orientation;
}

Eigen::Matrix3f Joint::getOrientation()
{
  return orientation_;n
}

void Joint::setPose(Pose joint_pose)
{
  position_ = joint_pose.position;
  orientation_ = joint_pose.orientation;
}

Pose Joint::getPose()
{
  Pose joint_pose;
  joint_pose.position = position_;
  joint_pose.orientation = orientation_;
  return joint_pose;
}
 

//////////////////////////////////////Tool////////////////////////////////////////////

Tool::Tool():
tool_type_("null"),
{
  position_from_final_joint_ = Eigen::Vector3f::Zero();
  orientation_from_final_joint_ = Eigen::Matrix3f::Identity(3,3);
  position_ = Eigen::Vector3f::Zero();
  orientation_ = Eigen::Matrix3f::Identity(3,3);
}

Tool::~Tool(){}

void init(String tool_type, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint)
{
  tool_type_ = tool_type;
  position_from_final_joint_ = position_from_final_joint;
  orientation_from_final_joint_ = orientation_from_final_joint;
}

Eigen::Vector3f Tool::getRelativePosition()
{
  return position_from_final_joint_;
}

Eigen::Matrix3f Tool::getRlativeOrientation()
{
  return orientation_from_final_joint_;
}

Pose Tool::getRelativePose()
{
  Pose temp;
  temp.position = position_from_final_joint_;
  temp.orientation = orientation_from_final_joint;
  return temp;
}

void Tool::setPosition(Eigen::Vector3f position)
{
  position_ = position;
}

void Tool::setOrientation(Eigen::Matrix3f orientation)
{
  orientation_ = orientation;
}

Eigen::Vector3f Tool::getPosition()
{
  return position_;
}

Eigen::Matrix3f Tool::getOrientation()
{
  return orientation_;
}

Pose Tool::getPose()
{
  Pose tool_pose;
  tool_pose.position = position_;
  tool_pose.orientation = orientation_;
  return tool_pose;
}




