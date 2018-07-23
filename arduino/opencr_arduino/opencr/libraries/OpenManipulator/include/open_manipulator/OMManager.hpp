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
  float angle;
  float velocity;
  float acceleration;
} JointState;

typedef struct
{
  Eigen::Vector3f position;
  Eigen::Vector3f orientation;
} Pose;

typedef struct
{
  Eigen::Vector3f linear_velocity;
  Eigen::Vector3f angular_velocity;
  Eigen::Vector3f linear_acceleration;
  Eigen::Vector3f angular_acceleration;
} DynamicPose

typedef struct
{
  Eigen::Vector3f relative_position;
  Eigen::Matrix3f relative_orientation;
} RelativePose;


class Link
{
 private:
  int8_t inner_control_point_size_;
  map<char*, RelativePose> inner_control_point_;

  char* name_;
  float mass_;
  Eigen::Matrix3f initial_inertia_tensor_; 
  RelativePose centor_of_mass_;

 public:
  Link():control_point_size_(1)
  {}
  ~Link(){}

  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(char* name, int8_t inner_control_point_size)
  {
    name_ = name;
    inner_control_point_size_=inner_control_point_size;
  }

  void addControlPoint(char* name, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
  {
    RelativePose temp;
    temp.relative_position = relative_position;
    temp.relative_orientation = relative_orientation;
    inner_control_point_.insert(pair<char*, RelativePose>(name,temp));
    if(inner_control_point_.size()>inner_control_point_size_)
    {
      cout << "error : control point size over in link [" << name_ << "]" << endl;
    }
  }

  void setCenterOfMass(float mass, Eigen::Matrix3f initial_inertia_tensor, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
  {
    mass_ = mass;
    initial_inertia_tensor_ = initial_inertia_tensor;
    centor_of_mass_.relative_position = relative_position;
    centor_of_mass_.relative_orientation = relative_orientation;
  }
  //////////////////////////////////////////////////////////////////////////

  /////////////////////////////Get fuction//////////////////////////////////
  int8_t getControlPointSize()
  {
    return inner_base_size_;
  }

  RelativePose getRelativePose(char* name)
  {
    return inner_control_point_.at(name);
  }

  RelativePose getRelativePose(char* to_name, char* from_name)
  {
    RelativePose result;

    result = inner_control_point_.at(to_name).relative_position - inner_control_point_.at(from_name).relative_position;
    result = inner_control_point_.at(from_name).relative_orientation.transpose() * inner_control_point_.at(to_name).relative_orientation;

    return result;
  }

  float getMass()
  {
    return mass_;
  }

  Eigen::Matrix3f getInitialInertiaTensor()
  {
    return initial_inertia_tensor_;
  } 

  RelativePose getRelativeCenterOfMassPose()
  {
    return centor_of_mass_;
  }

  RelativePose getRelativeCenterOfMassPose(char* from_name)
  {
    RelativePose result;

    result = centor_of_mass_.relative_position - inner_control_point_.at(from_name).relative_position;
    result = inner_control_point_.at(from_name).relative_orientation.transpose() * centor_of_mass_.relative_orientation;
    return result;
  }
  //////////////////////////////////////////////////////////////////////////
};

class ControlPoint
{
 private:
  Pose control_point_;
  DynamicPose dynamic_control_point_;

 public:
  ControlPoint()
  {
    control_point_.position = Eigen::Vector3f::Zero();
    control_point_.orientation = Eigen::Vector3f::Zero();
    dynamic_control_point_.linear_velocity = Eigen::Vector3f::Zero();
    dynamic_control_point_.angular_velocity = Eigen::Vector3f::Zero();
    dynamic_control_point_.linear_acceleration = Eigen::Vector3f::Zero();
    dynamic_control_point_.angular_acceleration = Eigen::Vector3f::Zero();
  }

  ~ControlPoint(){}

  ////////////////////////////////*Set fuction*///////////////////////////////
  void setPosition(Eigen::Vector3f position)
  {
    control_point_.position = position;
  }

  void setOrientation(Eigen::Vector3f orientation)
  {
    control_point_.orientation = orientation;
  }

  void setPose(Pose pose)
  {
    control_point_.position    = pose.position;
    control_point_.orientation = pose.orientation;
  }

  void setLinearVelocity(Eigen::Vector3f linear_velocity)
  {
    dynamic_control_point_.linear_velocity = linear_velocity;
  }

  void setAngularVelocity(Eigen::Vector3f angular_velocity)
  {
    dynamic_control_point_.angular_velocity = angular_velocity;
  }

  void setLinearAcceleration(Eigen::Vector3f linear_acceleration)
  {
    dynamic_control_point_.linear_acceleration = linear_acceleration;
  }

  void setAngularAcceleration(Eigen::Vector3f angular_acceleration)
  {
    dynamic_control_point_.angular_acceleration = angular_acceleration;
  }

  void setDynamicPose(DynamicPose dynamic_pose)
  {
    dynamic_control_point_.linear_velocity = dynamic_pose.linear_velocity;
    dynamic_control_point_.angular_velocity = dynamic_pose.angular_velocity;
    dynamic_control_point_.linear_acceleration = dynamic_pose.linear_acceleration;
    dynamic_control_point_.angular_acceleration = dynamic_pose.angular_acceleration;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
  Pose getPose()
  {
    return control_point_;
  }

  DynamicPose getDynamicPose()
  {
    return dynamic_control_point_;
  }
  ////////////////////////////////////////////////////////////////////////////
}

class Base: public ControlPoint
{
 private:
  char* name_;

 public:
  Base(){}
  ~Base(){}

  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(char* name, Eigen::Vector3f base_position, Eigen::Vector3f base_orientation)
  {
    name_ = name;
    control_point_.position = base_position;
    control_point_.orientation = base_orientation;
  }    
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
  char* getName()
  {
    return name_;
  }
  ////////////////////////////////////////////////////////////////////////////
};

class Mass: public ControlPoint
{
 private:
  char* name_;

 public:
  Mass(){}
  ~Mass(){}

  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(char* name)
  {
    name_ = name;
  }    
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
  char* getName()
  {
    return name_;
  }
  ////////////////////////////////////////////////////////////////////////////
}

class Joint: public ControlPoint
{
 private:
  char* name_;
  int8_t actuator_id_;
  Eigen::Vector3f axis_;
  JointState state_;

 public:
  Joint(): actuator_id_(-1)
  {
    axis_ = Eigen::Vector3f::Zero();
    state_.angle = 0.0;
    state_.velocity = 0.0;
    state_.acceleration = 0.0;
  }

  ~Joint(){}

  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(char* name, int8_t actuator_id, Eigen::Vector3f axis)
  {
    name_ = name;
    actuator_id_ = actuator_id;
    axis_ = axis;
  }    
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Set fuction*///////////////////////////////
  void setAngle(float angle)
  {
    state_.angle = angle;
  }

  void setAngularVelocity(float velocity)
  {
    state_.velocity = velocity;
  }

  void setAngularAcceleration(float acceleration)
  {
    state_.acceleration = acceleration;
  }

  void setJointState(JointState state)
  {
    state_.angle                = state.angle;
    state_.velocity             = state.velocity;
    state_.aacceleration        = state.acceleration;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
  char* getName()
  {
    return name_;
  }

  int8_t getActuatorId()
  {
    return actuator_id_;
  }

  Eigen::Vector3f getAxis()
  {
    return axis_;
  }

  float getAngle()
  {
    return state_.angle;
  }

  float getAngularVelocity()
  {
    return state_.angular_velocity;
  }

  float getAngularAcceleration()
  {
    return state_.angular_acceleration;
  }

  JointState getJointState()
  {
    return state_;
  }
  ////////////////////////////////////////////////////////////////////////////

class Tool: public ControlPoint
{
 private:
  char* name_;
  int8_t actuator_id_;
  Eigen::Vector3f axis_;

  bool on_off_;
  float actuator_value_;

 public:
  Tool(): actuator_id_(-1),
          on_off_(false),
          actuator_value_(0.0)
  {
    axis_ = Eigen::Vector3f::Zero();
  }

  ~Tool(){}

  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(char* name, int8_t actuator_id, Eigen::Vector3f axis)
  {
    name_ = name;
    actuator_id_ = actuator_id;
    axis_ = axis;
  }    
  ////////////////////////////////////////////////////////////////////////////
  
  ////////////////////////////////*Set fuction*///////////////////////////////
  void setOnOff(bool on_off)
  {
    on_off_=on_off;
  }

  void setActuatorValue(float actuator_value)
  {
    actuator_value_ = actuator_value;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
  char* getName()
  {
    return name_;
  }

  int8_t getActuatorId()
  {
    return actuator_id_;
  }

  Eigen::Vector3f getAxis()
  {
    return axis_;
  }

  bool getOnOff()
  {
    return on_off_;
  }

  float getActuatorValue()
  {
    return actuator_value_;
  }
  ////////////////////////////////////////////////////////////////////////////
};




template <int8_t JOINT_SIZE, int8_t LINK_SIZE, int8_t TOOL_SIZE>
class Manipulator
{
 private:
  int8_t type_ = TYPE;
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
  Manipulator(int8_t dof)
  {
    dof_ = dof;
  }

  ~Manipulator(){}

  void setDOF(int8_t dof)
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