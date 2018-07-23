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

  float mass_;
  Eigen::Matrix3f initial_inertia_tensor_; 
  RelativePose centor_of_mass_;

 public:
  Link():control_point_size_(1),
         mass_(0)
  {
    initial_inertia_tensor_ = Eigen::Matrix3f::Identity(3,3);
    centor_of_mass_.relative_position = Eigen::Vector3f::Zero();
    centor_of_mass_.relative_orientation = Eigen::Matrix3f::Identity(3,3);
  }
  ~Link(){}
  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(int8_t inner_control_point_size)
  {
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
    return inner_control_point_size_;
  }

  RelativePose getRelativePose(char* name)
  {
    if(inner_control_point_.find(name) != inner_control_point_.cend())
    {
      return inner_control_point_.at(name);
    }
    else
    {
      cout << "error : undefined control point (fuction : getRelativePose)" << endl;
      return NULL;
    }
  }

  RelativePose getRelativePose(char* to_name, char* from_name)
  {
    RelativePose result;
    if(inner_control_point_.find(to_name) != inner_control_point_.cend())
    {
      if(inner_control_point_.find(from_name) != inner_control_point_.cend())
      {
        result = inner_control_point_.at(to_name).relative_position - inner_control_point_.at(from_name).relative_position;
        result = inner_control_point_.at(from_name).relative_orientation.transpose() * inner_control_point_.at(to_name).relative_orientation;
        return result;
      }
      else
      {
        cout << "error : undefined control point (fuction : getRelativePose)" << endl;
        return NULL;
      }
    }
    else
    {
      cout << "error : undefined control point (fuction : getRelativePose)" << endl;
      return NULL;
    }
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

 public:
  Base(){}
  ~Base(){}
  ///////////////////////////*initialize fuction*/////////////////////////////
  void init(Eigen::Vector3f base_position, Eigen::Vector3f base_orientation)
  {
    control_point_.position = base_position;
    control_point_.orientation = base_orientation;
  }    
  ////////////////////////////////////////////////////////////////////////////

};

class Mass: public ControlPoint
{
 private:

 public:
  Mass(){}
  ~Mass(){}
}

class Joint: public ControlPoint
{
 private:
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
  void init(int8_t actuator_id, Eigen::Vector3f axis)
  {
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
  void init(int8_t actuator_id, Eigen::Vector3f axis)
  {
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

class Manipulator
{
 private:
  int8_t dof_;
  int8_t joint_size_;
  int8_t link_size_;
  int8_t tool_size_;

  map<char*, Base> base_;
  map<char*, Link> link_;
  map<char*, Mass> mass_;
  map<char*, Joint> joint_;
  map<char*, Tool> tool_;

 public:
  Manipulator():dof_(0),
                joint_size_(0),
                link_size_(0),
                tool_size_(0)
  {}
  ~Manipulator(){}
  ///////////////////////////*initialize fuction*/////////////////////////////
  void initManipulator(int8_t dof, int8_t joint_size, int8_t link_size, int8_t tool_size)
  {
    dof_=dof;
    joint_size_ = joint_size;
    link_size_ = link_size;
    tool_size_ = tool_size;
  }

  void makeBase(char* base_name, Eigen::Vector3f base_position, Eigen::Vector3f base_orientation)
  {
    Base base_temp;
    base_.insert(pair<char*, Base>(base_name,base_temp));
    base_.at(base_name).init(base_position, base_orientation);
    if(base_.size()>1)
    {
      cout << "error : base size over" << endl;
    }
  }

  void makeJoint(char* joint_name, int8_t actuator_id, Eigen::Vector3f axis)
  {
    Joint joint_temp;
    joint_.insert(pair<char*, Joint>(joint_name,joint_temp));
    joint_.at(joint_name).init(actuator_id, axis);
    if(joint_.size()>joint_size_)
    {
      cout << "error : joint size over"<< "(joint : " << joint_name << ")" << endl;
    }
  }

  void makeTool(char* tool_name, int8_t actuator_id, Eigen::Vector3f axis)
  {
    Tool tool_temp;
    tool_.insert(pair<char*, Tool>(tool_name,tool_temp));
    tool_.at(tool_name).init(actuator_id, axis);
    if(tool_.size()>tool_size_)
    {
      cout << "error : tool size over"<< "(tool : " << tool_name << ")" << endl;
    }
  }

  void makeLink(char* link_name, int8_t inner_control_point_size)
  {
    Link link_temp;
    link_.insert(pair<char*, Link>(link_name,link_temp));
    link_.at(link_name).init(inner_control_point_size);
    Mass mass_temp;
    mass_.insert(pair<char*, Mass>(link_name,mass_temp));
    if(link_.size()>link_size_)
    {
      cout << "error : link size over"<< "(link : " << link_name << ")" << endl;
    }
  }

  void addControlPoint(char* link_name, char* point_name, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
  {
    if(link_.find(link_name) != link_.cend())
    {
      if(findControlPoint(point_name)!=NULL)
      {
        link_.at(link_name).addControlPoint(point_name, relative_position, relative_orientation);
      }
      else
      {
        cout << "error : Unknown control point"<< " (point name : " << point_name << ", link_name : " << link_name << ")" << endl;
      }
    }
    else
    {
      cout << "error : added control point to unknown link"<< " (point name : " << point_name << ", link_name : " << link_name << ")" << endl;
    }
  }

  void setCenterOfMass(char* link_name, float mass, Eigen::Matrix3f initial_inertia_tensor, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
  {
    if(link_.find(link_name) != link_.cend())
    {
      link_.at(link_name).setCenterOfMass(mass, initial_inertia_tensor, relative_position, relative_orientation);
    }
    else
    {
      cout << "error : set center of mass to unknown link"<< " (link_name : " << link_name << ")" << endl;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Find fuction*//////////////////////////////
  char findControlPoint(char* point_name)
  {
    if(base_.find(point_name) != base_.cend())
    {
      return "b";
    }
    else if(joint_.find(point_name) != joint_.cend())
    {
      return "j";
    }
    else if(tool_.find(point_name) != tool_.cend())
    {
      return "t";
    }
    else if(mass_.find(point_name) != mass_.cend())
    {
      return "m";
    }
    else
    {
      return NULL;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Set fuction*///////////////////////////////
  void setPosition(char* name, Eigen::Vector3f position)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setPosition(position);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setPosition(position);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setPosition(position);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setPosition(position);
    }
    else
    {
      cout << "error" << endl;
    }
  }
  
  void setOrientation(char* name, Eigen::Vector3f orientation)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setOrientation(orientation);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setOrientation(orientation);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setOrientation(orientation);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setOrientation(orientation);
    }
    else
    {
      cout << "error" << endl;
    }
  }
  
  void setPose(char* name, Pose pose)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setPose(pose);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setPose(pose);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setPose(pose);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setPose(pose);
    }
    else
    {
      cout << "error" << endl;
    }
  }

  void setLinearVelocity(char* name, Eigen::Vector3f linear_velocity)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setLinearVelocity(linear_velocity);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setLinearVelocity(linear_velocity);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setLinearVelocity(linear_velocity);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setLinearVelocity(linear_velocity);
    }
    else
    {
      cout << "error" << endl;
    }
  }

  void setAngularVelocity(char* name, Eigen::Vector3f angular_velocity)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setAngularVelocity(angular_velocity);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setAngularVelocity(angular_velocity);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setAngularVelocity(angular_velocity);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setAngularVelocity(angular_velocity);
    }
    else
    {
      cout << "error" << endl;
    }
  }

  void setLinearAcceleration(char* name, Eigen::Vector3f linear_acceleration)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setLinearAcceleration(linear_acceleration);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setLinearAcceleration(linear_acceleration);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setLinearAcceleration(linear_acceleration);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setLinearAcceleration(linear_acceleration);
    }
    else
    {
      cout << "error" << endl;
    }
  }
  
  void setAngularAcceleration(char* name, Eigen::Vector3f angular_acceleration)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setAngularAcceleration(angular_acceleration);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setAngularAcceleration(angular_acceleration);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setAngularAcceleration(angular_acceleration);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setAngularAcceleration(angular_acceleration);
    }
    else
    {
      cout << "error" << endl;
    }
  }

  void setDynamicPose(char* name, DynamicPose dynamic_pose)
  {
    if(findControlPoint(name)=="b")
    {
      base_.at(name).setDynamicPose(dynamic_pose);
    }
    else if(findControlPoint(name)=="j")
    {
      joint_.at(name).setDynamicPose(dynamic_pose);
    }
    else if(findControlPoint(name)=="t")
    {
      tool_.at(name).setDynamicPose(dynamic_pose);
    }
    else if(findControlPoint(name)=="m")
    {
      mass_.at(name).setDynamicPose(dynamic_pose);
    }
    else
    {
      cout << "error" << endl;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get fuction*///////////////////////////////
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

  int8_t getControlPointSize()
  {
    return inner_control_point_size_;
  }

  RelativePose getRelativePose(char* link_name, char* name)
  {
    return link_.at(link_name).getRelativePose(name);
  }

  RelativePose getRelativePose(char* link_name, char* to_name, char* from_name)
  {
    return link_.at(link_name).getRelativePose(to_name, from_name);
  }

  float getMass(char* link_name)
  {
    return link_.at(link_name).getMass();
  }

  Eigen::Matrix3f getInitialInertiaTensor(char* link_name)
  {
    return link_.at(link_name).getInitialInertiaTensor();
  } 

  RelativePose getRelativeCenterOfMassPose(char* link_name)
  {
    return link_.at(link_name).getRelativeCenterOfMassPose();
  }

  RelativePose getRelativeCenterOfMassPose(char* link_name, char* from_name)
  {
    return link_.at(link_name).getRelativeCenterOfMassPose(from_name);
  }

  Pose getPose(char* name)
  {
    if(findControlPoint(name)=="b")
    {
      return base_.at(name).getPose();
    }
    else if(findControlPoint(name)=="j")
    {
      return joint_.at(name).getPose();
    }
    else if(findControlPoint(name)=="t")
    {
      return tool_.at(name).getPose();
    }
    else if(findControlPoint(name)=="m")
    {
      return mass_.at(name).getPose();
    }
    else
    {
      cout << "error" << endl;
    }
  }

  DynamicPose getDynamicPose(char* name)
  {
    if(findControlPoint(name)=="b")
    {
      return base_.at(name).getDynamicPose();
    }
    else if(findControlPoint(name)=="j")
    {
      return joint_.at(name).getDynamicPose();
    }
    else if(findControlPoint(name)=="t")
    {
      return tool_.at(name).getDynamicPose();
    }
    else if(findControlPoint(name)=="m")
    {
      return mass_.at(name).getDynamicPose();
    }
    else
    {
      cout << "error" << endl;
    }
  }
  ////////////////////////////////////////////////////////////////////////////




};

#endif // OMMANAGER_HPP_