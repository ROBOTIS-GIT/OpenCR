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

#ifndef OMMANAGER_HPP_
#define OMMANAGER_HPP_

#include "OMDebug.hpp"

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>
#include <vector>       

using namespace std;

typedef struct
{
  Eigen::Vector3f position;
  Eigen::Matrix3f orientation;
} Pose;

typedef struct
{
  float angle;
  float angular_velocity;
  float angular_acceleration;
} State;

typedef struct
{
  int8_t joint_number;
  Eigen::Vector3f relative_position;
  Eigen::Matrix3f relative_orientation;
} InnerJoint;

typedef struct
{
  float mass;
  Eigen::Vector3f relative_center_position;
  float moment;
} Inertia;

class Joint
{
 private:
  int8_t dxl_id_;
  Eigen::Vector3f axis_;
  State joint_state_;
  Pose joint_pose_;

 public:
  /////////////////func///////////////////
  Joint(): 
  dxl_id_(-1),
  {
    joint_state_.angle = 0.0;
    joint_state_.angular_velocity = 0.0;
    joint_state_.angular_acceleration = 0.0;
    joint_pose_.position = Eigen::Vector3f::Zero();
    joint_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
    axis_ = Eigen::Vector3f::Zero();
  }

  ~Joint(){}

  void init(int8_t dxl_id, Eigen::Vector3f axis)
  {
    dxl_id_ = dxl_id;
    axis_ = axis;
  }    
  
  int8_t getId()
  {
    return dxl_id_;
  }
  Eigen::Vector3f getAxis()
  {
    return axis_;
  }

  void setAngle(float angle)
  {
    joint_state_.angle = angle;
  }

  void setAngularVelocity(float angular_velocity)
  {
    joint_state_.angular_velocity = angular_velocity;
  }

  void setAngularAcceleration(float angular_acceleration)
  {
    joint_state_.angular_acceleration = angular_acceleration;
  }

  void setJointState(State joint_state)
  {
    joint_state_.angle = joint_state.angle;
    joint_state_.angular_velocity = joint_state.angular_velocity;
    joint_state_.angular_acceleration = joint_state.angular_acceleration;
  }

  float getAngle()
  {
    return joint_state_.angle_;
  }

  float getAngularVelocity()
  {
    return joint_state_.angular_velocity;
  }

  float getAngularAcceleration()
  {
    return joint_state_.angular_acceleration;
  }

  State getJointState()
  {
    State temp; 
    temp.angle = joint_state_.angle;
    temp.angular_velocity = joint_state_.angular_velocity;
    temp.angular_acceleration = joint_state_.angular_acceleration;
    return temp;
  }

  void setPosition(Eigen::Vector3f position)
  {
    joint_pose_.position = position;
  }

  void setOrientation(Eigen::Matrix3f orientation)
  {
    joint_pose_.orientation = orientation;
  }

  void setPose(Pose joint_pose)
  {
    joint_pose_.position = joint_pose.position;
    joint_pose_.orientation = joint_pose.orientation;
  }

  Eigen::Vector3f getPosition()
  {
    return joint_pose_.position;
  }

  Eigen::Matrix3f getOrientation()
  {
    return joint_pose_.orientation;
  }

  Pose getPose()
  {
    Pose temp;
    temp.position = joint_pose_.position;
    temp.orientation = joint_pose_.orientation;
    return temp;
  }
  ////////////////////////////////////////
};

class Link
{
 private:
  int8_t inner_joint_size_;
  Inertia inertia_;
  vector<InnerJoint> inner_joint_;

  Eigen::Vector3f center_position_;
 public:
  /////////////////func///////////////////
  Link():
  inner_joint_size_(1),
  {
    inertia_.mass = 0;
    inertia_.center_position = Eigen::Vector3f::Zero();
    inertia_.moment = 0;
  }

  ~Link(){}

  void init(int8_t inner_joint_size)
  {
    inner_joint_size_ = inner_joint_size;
    inner_joint_.resize(inner_joint_size_);
  }
  
  void setInnerJoint(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
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

  int8_t getInnerJointSize()
  {
    return inner_joint_size_;
  }

  InnerJoint getInnerJointInformation(int8_t joint_number)
  {
    InnerJoint temp;
    temp.joint_number = inner_joint_.at(findJoint(joint_number)).joint_number;
    temp.relative_position = inner_joint_.at(findJoint(joint_number)).relative_position;
    temp.relative_orientation = inner_joint_.at(findJoint(joint_number)).relative_orientation;
    return temp;
  }

  Eigen::Vector3f getRelativeJointPosition(int8_t to, int8_t from)
  {
    Eigen::Vector3f temp;
    temp = inner_joint_.at(FindJoint(to)).relative_position - inner_joint_.at(FindJoint(from)).relative_position;
    return temp; 
  }

  Eigen::Matrix3f getRelativeJointOrientation(int8_t to, int8_t from)
  {
    Eigen::Matrix3f temp;
    temp = inner_joint_.at(FindJoint(from)).relative_orientation.transpose() * inner_joint_.at(FindJoint(to)).relative_orientation;
    return temp; 
  }

  Eigen::Vector3f getRelativeJointPosition(int8_t joint_number)
  {
    Eigen::Vector3f temp;
    temp = inner_joint_.at(FindJoint(joint_number)).relative_position;
    return temp; 
  }

  Eigen::Matrix3f getRelativeJointOrientation(int8_t joint_number)
  {
    Eigen::Matrix3f temp;
    temp = inner_joint_.at(FindJoint(joint_number)).relative_orientation;
    return temp; 
  }
  
  int8_t findJoint(int8_t joint_number)
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

  void setCenterPosition(Eigen::Vector3f center_position)
  {
    center_position_ = center_position;
  }

    Eigen::Vector3f getCenterPosition()
  {
    return center_position_;
  }

  void setInertia(Inertia inertia)
  {
    inertia_ = inertia;
  }

  void setMass(float mass)
  {
    inertia_.mass = mass;
  }

  void setRelativeCenterPosition(Eigen::Vector3f relative_center_position)
  {
    inertia_.center_position =center_position;
  }

  void setInertiaMoment(float inertia_moment)
  {
    inertia_.moment = inertia_moment;
  }

  Inertia getInertia()
  {
    return inertia_;
  }

  float getMass()
  {
    return inertia_.mass;
  }

  Eigen::Vector3f getRelativeCenterPosition()
  {
    return inertia_.center_position;
  }

  Eigen::Vector3f getRelativeCenterPosition(int8_t from)
  {
    Eigen::Vector3f temp;
    temp = inertia_.center_position - inner_joint_.at(FineJoint(from)).relative_position;
    return temp;
  }

  float getInertiaMoment()
  {
    return inertia_.moment;
  }


  ////////////////////////////////////////
};

class Base: public Link
{
 private:
  Pose relative_base_pose_;
    
  Pose base_pose_;
 public:
  /////////////////func///////////////////
  Base(): 
  {
    base_pose_.position = Eigen::Vector3f::Zero();
    base_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
    relative_base_pose_.position = Eigen::Vector3f::Zero();
    relative_base_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
  }
    ~Base(){}

  void setRelativeBasePostion(Eigen::Vector3f relative_base_position)
  {
    relative_base_pose_.position = relative_base_position;
  }

  void setRelativeBaseOrientation(Eigen::Matrix3f relative_base_orientation)
  {
    relative_base_pose_.orientation = relative_base_orientation;
  }

  void setRelativeBasePose(Pose relative_base_pose)
  {
    relative_base_pose_.position = relative_base_pose.position;
    relative_base_pose_.orientation = relative_base_pose.orientation;
  }

  void setPosition(Eigen::Vector3f base_position)
  {
    base_pose_.position = base_position;
  }

  void setOrientation(Eigen::Matrix3f base_orientation)
  {
    base_pose_.orientation = base_orientation;
  }

  void setPose(Pose base_pose)
  {
    base_pose_.position = base_pose.position;
    base_pose_.orientation = base_pose.orientation;
  }

  Eigen::Vector3f getPosition()
  {
    return base_pose_.position;
  }

  Eigen::Matrix3f getOrientation()
  {
    return base_pose_.orientation;
  }

  Pose getPose()
  {
    Pose temp;
    temp.position = base_pose_.position;
    temp.orientation = base_pose_.orientation;
    return temp;
  }

  Eigen::Vector3f getRelativeBaseJointPosition(int8_t to)
  {
    Eigen::Vector3f temp;
    temp = getRelativeJointPosition(to) - relative_base_pose_.position;
    return temp; 
  }

  Eigen::Matrix3f getRelativeBaseJointOrientation(int8_t to)
  {
    Eigen::Matrix3f temp;
    temp = relative_base_pose_.orientation.transpose() * getRelativeJointorientation(to);
    return temp; 
  }
    ////////////////////////////////////////
};

class Tool: public Link
{
 private:
  int8_t tool_type_;
  Pose relative_tool_pose_;
  
  Pose tool_pose_;
 public:
  /////////////////func///////////////////
  Tool():
  tool_type_(0)
  {
    tool_pose_.position = Eigen::Vector3f::Zero();
    tool_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
    relatvie_tool_pose_.position = Eigen::Vector3f::Zero();
    relatvie_tool_pose_.orientation = Eigen::Matrix3f::Identity(3,3);
  }

  ~Tool(){}

  void setToolType(int8_t tool_type)
  {
    tool_type_ = tool_type;
  }

  int8_t getToolType()
  {
    return tool_type_;
  }

  void setRelativeToolPosition(Eigen::Vector3f relative_tool_position)
  {
    relatvie_tool_pose_.position = relatvie_tool_position;
  }

  void setRelativeToolOrientation(Eigen::Matrix3f relatvie_tool_orientation)
  {
    relatvie_tool_pose_.orientation = relatvie_tool_orientation;
  }

  void setRelativeToolPose(Pose relative_tool_pose)
  {
    relatvie_tool_pose_.position = relatvie_tool_pose.position;
    relatvie_tool_pose_.orientation = relatvie_tool_pose.orientation;
  }

  void setPosition(Eigen::Vector3f tool_position)
  {
    tool_pose_.position = tool_position;
  }

  void setOrientation(Eigen::Matrix3f tool_orientation)
  {
    tool_pose_.orientation = tool_orientation;
  }

  void setPose(Pose tool_pose)
  {
    tool_pose_.position = tool_pose.position;
    tool_pose_.orientation = tool_pose.orientation;
  }

  Eigen::Vector3f getPosition()
  {
    return tool_pose_.position;
  }

  Eigen::Matrix3f getOrientation()
  {
    return tool_pose_.orientation;
  }

  Pose getPose()
  {
    Pose temp;
    temp.position = tool_pose_.position;
    temp.orientation = tool_pose_.orientation;
    return temp;
  }

  Eigen::Vector3f getRelativeToolPosition(int8_t from)
  {
    Eigen::Vector3f temp;
    temp = relative_tool_pose_.position - getRelativeJointPosition(from);
    return temp; 
  }

  Eigen::Matrix3f getRelativeToolOrientation(int8_t from)
  {
    Eigen::Matrix3f temp;
    temp = getRelativeJointorientation(from).transpose() * relative_tool_pose_.orientation;
    return temp; 
  }
  ////////////////////////////////////////
};

template <int8_t TYPE, int8_t JOINT_SIZE, int8_t LINK_SIZE, int8_t TOOL_SIZE>
class Manipulator
{
 private:
  inte8_t type_ = TYPE;
  int8_t dof_;
  int8_t joint_size_ = JOINT_SIZE;
  int8_t link_size_ = LINK_SIZE;
  int8_t tool_size_ = TOOL_SIZE;

 public:
  Base base_;
  Joint joint_[JOINT_SIZE];
  Link link_[LINK_SIZE];
  Tool tool_[TOOL_SIZE];

  /////////////////func///////////////////
  Manipulator(int8_t dof):
  name("UnknownManipulator")
  {
    dof_ = dof;
  }
  ~Manipulator(){}

  void setDOF(int8_t dof);
  {
    dof_=dof;
  }

  int8_t getDOF()
  {
    return dof_;
  }

  int8_t getJointSize()
  {
    return joint_size_;
  }

  int8_t getLinkSize()
  {
    return link_size_;
  }

  int8_t getToolSize()
  {
    return tool_size_;
  }   

////////////////////////////////////////
};

#endif // OMMANAGER_HPP_