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

/* Authors: Hye-Jong KIM, Darby Lim */

#ifndef OMMANAGER_HPP_
#define OMMANAGER_HPP_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>
#include <map>     
#include <vector>     

#include "OMDebug.hpp"

using namespace std, Eigen;

typedef int8_t NAME

typedef struct
{
  Vector3f position;
  Matrix3f orientation;
} Pose;

typedef struct
{
  Vector6f velocity;
  Vector6f acceleration;
} State;

typedef struct
{
  int8_t id;

  bool on_off;
  float value;   //mm or rad
} Actuator;

typedef struct
{
  Vector3f axis;
  float angle;
  float velocity;
  float acceleration;

  Actuator actuator; 
} Joint;

typedef struct
{
  float mass;
  Matrix3f inertia_tensor;
  Pose center_of_mass;
} Inertia;

typedef struct
{
  NAME me_;           
  NAME parent_;
  vector<NAME> child_;

  Pose relative_to_parent_;

  Pose pose_to_world_;
  State origin_;
  
  Joint joint_;

  Inertia inertia_;  
} Component;


typedef struct
{
  NAME name_;           
  NAME child_;

  Pose pose_;
  State origin_;
} World;

class Manipulator
{
 private: 
  int dof_;
  int8_t component_size_;

  World world_;
  map<Name, Component> component_;

 public:
  Manipulator()
  {
    dof_ = 0;
    component_size_ = 0;
  }
  ~Manipulator(){}
  //virtual ~Manipulator(){};
  //////////////////////////////*Parameter list*///////////////////////////////
  /*
  dof_
  component_size_
  world_.name_
  world_.child_
  world_.pose_.position
  world_.pose_.orientation
  world_.origin_.velocity
  world_.origin_.acceleration
  component_.at(name).parent_
  component_.at(name).child_(i)
  component_.at(name).relative_to_parent_.position
  component_.at(name).relative_to_parent_.orientation
  component_.at(name).pose_to_world_.position
  component_.at(name).pose_to_world_.orientation
  component_.at(name).origin_.velocity
  component_.at(name).origin_.acceleration
  component_.at(name).joint_.axis
  component_.at(name).joint_.angle
  component_.at(name).joint_.velocity
  component_.at(name).joint_.acceleration
  component_.at(name).joint_.actuator.id
  component_.at(name).joint_.actuator.on_off
  component_.at(name).joint_.actuator.value
  component_.at(name).inertia_.mass
  component_.at(name).inertia_.inertia_tensor
  component_.at(name).inertia_.center_of_mass.position
  component_.at(name).inertia_.center_of_mass.orientation
  */
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////*initialize function*/////////////////////////////
  void init(int dof, int component_size, bool *error);
  // void addWorld(Name world_name, NAME child_name, Vector3f world_position = Vector3f::Zero(), Matrix3f world_orientation = Matrix3f::Identity(3,3), bool *error)
  void addComponent(Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error);
  void addComponentChild(Name me_name, NAME child_name, bool *error);
  void checkManipulatorSetting(bool *error);
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Set function//////////////////////////////////
  // void setWorldPose(Pose world_pose, bool *error);
  // void setWorldPosition(Vector3f world_position, bool *error);
  // void setWorldOrientation(Matrix3f world_orientation, bool *error);
  // void setWorldState(State world_state, bool *error);
  // void setWorldVelocity(Vector6f world_velocity, bool *error);
  // void setWorldAcceleration(Vector6f world_acceleration, bool *error);
  void setComponent(Name name, Component component, bool *error);
  void setComponentPoseToWorld(Name name Pose pose_to_world, bool *error);
  void setComponentPositionToWorld(Name name, Vector3f position_to_world, bool *error);
  void setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd, bool *error);
  void setComponentStateToWorld(Name name State state_to_world, bool *error);
  void setComponentVelocityToWorld(Name name, Vector6f velocity, bool *error);
  void setComponentAccelerationToWorld(Name name, Vector6f accelaration, bool *error);
  void setComponentJointAngle(Name name, float angle, bool *error);
  void setComponentJointVelocity(Name name, float angular_velocity, bool *error);
  void setComponentJointAcceleration(Name name, float angular_acceleration, bool *error);
  void setComponentActuatorOnOff(Name name, bool on_off, bool *error);
  void setComponentActuatorValue(Name name, float actuator_value, bool *error);
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////Get function//////////////////////////////////
  int getDOF(bool *error);
  int8_t getComponentSize(bool *error);
  // Name getWorldName(bool *error);
  // Name getWorldChildName(bool *error);
  // Pose getWorldPose(bool *error);
  // Vector3f getWorldPosition(bool *error);
  // Matrix3f getWorldOrientation(bool *error);
  // State getWorldState(bool *error);
  // Vector6f getWorldVelocity(bool *error);
  // Vector6f getWorldAcceleration(bool *error);
  Component getComponent(Name name, bool *error);
  Name getComponentParentName(Name name, bool *error);
  vector<NAME> getComponentChildName(Name name, bool *error);
  Pose getComponentPoseToWorld(Name name, bool *error);
  Vector3f getComponentPositionToWorld(Name name, bool *error);
  Matrix3f getComponentOrientationToWorld(Name name, bool *error);
  State getComponentStateToWorld(Name name, bool *error);
  Vector6f getComponentVelocityToWorld(Name name, bool *error);
  Vector6f getComponentAccelerationToWorld(Name name, bool *error);
  Pose getComponentRelativePoseToParent(Name name, bool *error);
  Vector3f getComponentRelativePositionToParent(Name name, bool *error);
  Matrix3f getComponentRelativeOrientationToParent(Name name, bool *error);
  Joint getComponentJoint(Name name, bool *error);
  Vector3f getComponentJointAxis(Name name, bool *error);
  float getComponentJointAngle(Name name, bool *error);
  float getComponentJointVelocity(Name name, bool *error);
  float getComponentJointAcceleration(Name name, bool *error);
  int8_t getComponentActuatorId(Name name, bool *error);
  bool getComponentActuatorOnOff(Name name, bool *error);
  float getComponentActuatorValue(Name name, bool *error);
  float getComponentMass(Name name, bool *error);
  Matrix3f getComponentInertiaTensor(Name name, bool *error);
  Pose getComponentCenterOfMassPose(Name name, bool *error);
  Vector3f getComponentCenterOfMassPosition(Name name, bool *error);
  Matrix3f getComponentCenterOfMassOrientation(Name name, bool *error);
  /////////////////////////////////////////////////////////////////////////////
};

///////////////////////////*initialize function*/////////////////////////////
void Manipulator::init(int dof, int component_size, bool *error)
{
  dof_ = dof;
  component_size_ = component_size;
}

void Manipulator::addComponent(Name me_name, Name parent_name, NAME child_name, Vector3f relative_position, Matrix3f relative_orientation, int8_t actuator_id = -1, Vector3f axis_of_rotation = Vector3f::Zero(), float mass = 0.0, Matrix3f inertia_tensor = Matrix3f::Identity(3,3), Vector3f center_of_mass = Vector3f::Zero(), bool *error)
{

}

void Manipulator::addComponentChild(Name me_name, NAME child_name, bool *error)
{

}

void Manipulator::checkManipulatorSetting(bool *error)
{

}

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setComponent(Name name, Component component, bool *error)
{

}

void Manipulator::setComponentPoseToWorld(Name name, Pose pose_to_world, bool *error)
{

}

void Manipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world, bool *error)
{

}

void Manipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd, bool *error)
{

}

void Manipulator::setComponentStateToWorld(Name name, State state_to_world, bool *error)
{

}

void Manipulator::setComponentVelocityToWorld(Name name, Vector6f velocity, bool *error)
{

}

void Manipulator::setComponentAccelerationToWorld(Name name, Vector6f accelaration, bool *error)
{

}

void Manipulator::setComponentJointAngle(Name name, float angle, bool *error)
{

}

void Manipulator::setComponentJointVelocity(Name name, float angular_velocity, bool *error)
{

}

void Manipulator::setComponentJointAcceleration(Name name, float angular_acceleration, bool *error)
{

}

void Manipulator::setComponentActuatorOnOff(Name name, bool on_off, bool *error)
{

}

void Manipulator::setComponentActuatorValue(Name name, float actuator_value, bool *error)
{

}

/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Get function//////////////////////////////////
int Manipulator::getDOF(, bool *error)
{

}

int8_t Manipulator::getComponentSize(, bool *error)
{

}

Component Manipulator::getComponent(Name name, bool *error)
{

}

Name Manipulator::getComponentParentName(Name name, bool *error)
{

}

vector<NAME> Manipulator::getComponentChildName(Name name, bool *error)
{

}

Pose Manipulator::getComponentPoseToWorld(Name name, bool *error)
{

}

Vector3f Manipulator::getComponentPositionToWorld(Name name, bool *error)
{

}

Matrix3f Manipulator::getComponentOrientationToWorld(Name name, bool *error)
{

}

State Manipulator::getComponentStateToWorld(Name name, bool *error)
{

}

Vector6f Manipulator::getComponentVelocityToWorld(Name name, bool *error)
{

}

Vector6f Manipulator::getComponentAccelerationToWorld(Name name, bool *error)
{

}

Pose Manipulator::getComponentRelativePoseToParent(Name name, bool *error)
{

}

Vector3f Manipulator::getComponentRelativePositionToParent(Name name, bool *error)
{

}

Matrix3f Manipulator::getComponentRelativeOrientationToParent(Name name, bool *error)
{

}

Joint Manipulator::getComponentJoint(Name name, bool *error)
{

}

Vector3f Manipulator::getComponentJointAxis(Name name, bool *error)
{

}

float Manipulator::getComponentJointAngle(Name name, bool *error)
{

}

float Manipulator::getComponentJointVelocity(Name name, bool *error)
{

}

float Manipulator::getComponentJointAcceleration(Name name, bool *error)
{

}

int8_t Manipulator::getComponentActuatorId(Name name, bool *error)
{

}

bool Manipulator::getComponentActuatorOnOff(Name name, bool *error)
{

}

float Manipulator::getComponentActuatorValue(Name name, bool *error)
{

}

float Manipulator::getComponentMass(Name name, bool *error)
{

}

Matrix3f Manipulator::getComponentInertiaTensor(Name name, bool *error)
{

}

Pose Manipulator::getComponentCenterOfMassPose(Name name, bool *error)
{

}

Vector3f Manipulator::getComponentCenterOfMassPosition(Name name, bool *error)
{

}

Matrix3f Manipulator::getComponentCenterOfMassOrientation(Name name, bool *error)
{

}

/////////////////////////////////////////////////////////////////////////////



#if 0
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
  Eigen::Matrix3f orientation;
} Pose;

typedef struct
{
  Eigen::Vector3f linear_velocity;
  Eigen::Vector3f angular_velocity;
  Eigen::Vector3f linear_acceleration;
  Eigen::Vector3f angular_acceleration;
} DynamicPose;

typedef struct
{
  Eigen::Vector3f relative_position;
  Eigen::Matrix3f relative_orientation;
} RelativePose;

class Link
{
 private:
  int8_t inner_control_point_size_;
  std::map<char*, RelativePose> inner_control_point_;

  float mass_;
  Eigen::Matrix3f initial_inertia_tensor_; 
  RelativePose centor_of_mass_;

 public:
  Link()
  {
    inner_control_point_size_ = 1;
    mass_ = 0;
    initial_inertia_tensor_ = Eigen::Matrix3f::Identity(3,3);
    centor_of_mass_.relative_position = Eigen::Vector3f::Zero();
    centor_of_mass_.relative_orientation = Eigen::Matrix3f::Identity(3,3);
  }
  ~Link(){}
  ///////////////////////////*initialize function*/////////////////////////////
  void init(int8_t inner_control_point_size, bool* error = false)
  {
    inner_control_point_size_=inner_control_point_size;
  }

  void addControlPoint(char* name, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation, bool* error = false)
  {
    RelativePose temp;
    temp.relative_position = relative_position;
    temp.relative_orientation = relative_orientation;
    inner_control_point_[name] = temp;
    if(inner_control_point_.size()>inner_control_point_size_)
    {
      // cout << "error : control point size over in link [" << name_ << "]" << endl;
      *error = true;
    }
  }

  void setCenterOfMass(float mass, Eigen::Matrix3f initial_inertia_tensor, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation, bool* error = false)
  {
    mass_ = mass;
    initial_inertia_tensor_ = initial_inertia_tensor;
    centor_of_mass_.relative_position = relative_position;
    centor_of_mass_.relative_orientation = relative_orientation;
  }
  //////////////////////////////////////////////////////////////////////////

  /////////////////////////////Get function//////////////////////////////////
  int8_t getControlPointSize(bool* error = false)
  {
    return inner_control_point_size_;
  }

  vector<char*> getLinkControlPointNameList(bool* error = false)/////////////////////***********************
  {
    vector<char*> name_list;std::
    map<char*, RelativePose>::iterator control_point_iterator;

    for(control_point_iterator = inner_control_point_.begin(); control_point_iterator != inner_control_point_.end(); control_point_iterator++)
    {
      name_list.push_back(control_point_iterator->first);
    }
    return name_list;
  }

  RelativePose getRelativePose(char* name, bool* error = false)
  {
    if(inner_control_point_.find(name) != inner_control_point_.end())
    {
      return inner_control_point_.at(name);
    }
    else
    {
      // cout << "error : undefined control point (function : getRelativePose)" << endl;
      *error = true;
    }
  }

  RelativePose getRelativePose(char* to_name, char* from_name, bool* error = false)
  {
    RelativePose result;
    if(inner_control_point_.find(to_name) != inner_control_point_.end())
    {
      if(inner_control_point_.find(from_name) != inner_control_point_.end())
      {
        result.relative_position = inner_control_point_.at(to_name).relative_position - inner_control_point_.at(from_name).relative_position;
        result.relative_orientation = inner_control_point_.at(from_name).relative_orientation.transpose() * inner_control_point_.at(to_name).relative_orientation;
        return result;
      }
      else
      {
        // cout << "error : undefined control point (function : getRelativePose)" << endl;
        *error = true;
      }
    }
    else
    {
      // cout << "error : undefined control point (function : getRelativePose)" << endl;
    *error = true;
    }
  }

  float getMass(bool* error = false)
  {
    return mass_;
  }

  Eigen::Matrix3f getInitialInertiaTensor(bool* error = false)
  {
    return initial_inertia_tensor_;
  } 

  RelativePose getRelativeCenterOfMassPose(bool* error = false)
  {
    return centor_of_mass_;
  }

  RelativePose getRelativeCenterOfMassPose(char* from_name, bool* error = false)
  {
    RelativePose result;
    if(inner_control_point_.find(from_name) != inner_control_point_.end())
    {
      result.relative_position = centor_of_mass_.relative_position - inner_control_point_.at(from_name).relative_position;
      result.relative_orientation = inner_control_point_.at(from_name).relative_orientation.transpose() * centor_of_mass_.relative_orientation;
      return result;
    }
    else
    {
      // cout << "error : undefined control point (function : getRelativePose)" << endl;
      *error = true;
    }    
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
    control_point_.orientation = Eigen::Matrix3f::Identity(3,3);
    dynamic_control_point_.linear_velocity = Eigen::Vector3f::Zero();
    dynamic_control_point_.angular_velocity = Eigen::Vector3f::Zero();
    dynamic_control_point_.linear_acceleration = Eigen::Vector3f::Zero();
    dynamic_control_point_.angular_acceleration = Eigen::Vector3f::Zero();
  }
  ~ControlPoint(){}
  ////////////////////////////////*Set function*///////////////////////////////
  void setPosition(Eigen::Vector3f position, bool* error = false)
  {
    control_point_.position = position;
  }

  void setOrientation(Eigen::Matrix3f orientation, bool* error = false)
  {
    control_point_.orientation = orientation;
  }

  void setPose(Pose pose, bool* error = false)
  {
    control_point_ = pose;
  }

  void setLinearVelocity(Eigen::Vector3f linear_velocity, bool* error = false)
  {
    dynamic_control_point_.linear_velocity = linear_velocity;
  }

  void setAngularVelocity(Eigen::Vector3f angular_velocity, bool* error = false)
  {
    dynamic_control_point_.angular_velocity = angular_velocity;
  }

  void setLinearAcceleration(Eigen::Vector3f linear_acceleration, bool* error = false)
  {
    dynamic_control_point_.linear_acceleration = linear_acceleration;
  }

  void setAngularAcceleration(Eigen::Vector3f angular_acceleration, bool* error = false)
  {
    dynamic_control_point_.angular_acceleration = angular_acceleration;
  }

  void setDynamicPose(DynamicPose dynamic_pose, bool* error = false)
  {
    dynamic_control_point_ = dynamic_pose;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get function*///////////////////////////////
  Pose getPose(bool* error = false)
  {
    return control_point_;
  }

  DynamicPose getDynamicPose(bool* error = false)
  {
    return dynamic_control_point_;
  }
  ////////////////////////////////////////////////////////////////////////////
};

class Base: public ControlPoint
{
 private:

 public:
  Base(){}
  ~Base(){}
  ///////////////////////////*initialize function*/////////////////////////////
  void init(Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation, bool* error = false)
  {
    setPosition(base_position);
    setOrientation(base_orientation);
  }    
  ////////////////////////////////////////////////////////////////////////////
};

class Mass: public ControlPoint
{
 private:

 public:
  Mass(){}
  ~Mass(){}
};

class Joint: public ControlPoint
{
 private:
  int8_t actuator_id_;
  Eigen::Vector3f axis_;
  JointState state_;

 public:
  Joint()
  {
    actuator_id_ = -1;
    axis_ = Eigen::Vector3f::Zero();
    state_.angle = 0.0;
    state_.velocity = 0.0;
    state_.acceleration = 0.0;
  }
  ~Joint(){}
  ///////////////////////////*initialize function*/////////////////////////////
  void init(int8_t actuator_id = -1, Eigen::Vector3f axis = Eigen::Vector3f::Zero(), bool* error = false)
  {
    actuator_id_ = actuator_id;
    axis_ = axis;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Set function*///////////////////////////////
  void setAngle(float angle, bool* error = false)
  {
    state_.angle = angle;
  }

  void setAngularVelocity(float velocity, bool* error = false)
  {
    state_.velocity = velocity;
  }

  void setAngularAcceleration(float acceleration, bool* error = false)
  {
    state_.acceleration = acceleration;
  }

  void setJointState(JointState state, bool* error = false)
  {
    state_ = state;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get function*///////////////////////////////
  int8_t getActuatorId(bool* error = false)
  {
    return actuator_id_;
  }

  Eigen::Vector3f getAxis(bool* error = false)
  {
    return axis_;
  }

  float getAngle(bool* error = false)
  {
    return state_.angle;
  }

  float getAngularVelocity(bool* error = false)
  {
    return state_.velocity;
  }

  float getAngularAcceleration(bool* error = false)
  {
    return state_.acceleration;
  }

  JointState getJointState(bool* error = false)
  {
    return state_;
  }
  ////////////////////////////////////////////////////////////////////////////
};

class Tool: public ControlPoint
{
 private:
  int8_t actuator_id_;
  Eigen::Vector3f axis_;

  bool on_off_;
  float actuator_value_;

 public:
  Tool()
  {
    actuator_id_ = -1;
    on_off_ = false;
    actuator_value_ = 0.0;
    axis_ = Eigen::Vector3f::Zero();
  }
  ~Tool(){}
  ///////////////////////////*initialize function*/////////////////////////////
  void init(int8_t actuator_id, Eigen::Vector3f axis, bool* error = false)
  {
    actuator_id_ = actuator_id;
    axis_ = axis;
  }    
  ////////////////////////////////////////////////////////////////////////////
  
  ////////////////////////////////*Set function*///////////////////////////////
  void setOnOff(bool on_off, bool* error = false)
  {
    on_off_=on_off;
  }

  void setActuatorValue(float actuator_value, bool* error = false)
  {
    actuator_value_ = actuator_value;
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get function*///////////////////////////////
  int8_t getActuatorId(bool* error = false)
  {
    return actuator_id_;
  }

  Eigen::Vector3f getAxis(bool* error = false)
  {
    return axis_;
  }

  bool getOnOff(bool* error = false)
  {
    return on_off_;
  }

  float getActuatorValue(bool* error = false)
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

  std::map<char*, Base> base_;
  std::map<char*, Link> link_;
  std::map<char*, Mass> mass_;
  std::map<char*, Joint> joint_;
  std::map<char*, Tool> tool_;

 public:
  Manipulator()
  {
    dof_ = 0;
    joint_size_ = 0;
    link_size_ = 0;
    tool_size_ = 0;
  }
  ~Manipulator(){}
  ////////////////////////////////*Find function*//////////////////////////////
  ControlPoint* findControlPoint(char* point_name, bool* error = false)
  {
    if(base_.find(point_name) != base_.end())
    {
      return (ControlPoint*) &base_[point_name];
    }
    else if(joint_.find(point_name) != joint_.end())
    {
      return (ControlPoint*) &joint_[point_name];
    }
    else if(tool_.find(point_name) != tool_.end())
    {
      return (ControlPoint*) &tool_[point_name];
    }
    else if(mass_.find(point_name) != mass_.end())
    {
      return (ControlPoint*) &mass_[point_name];
    }
    else
    {
      // cout << "error" << endl;
      *error = true;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ///////////////////////////*initialize function*/////////////////////////////
  void initManipulator(int8_t dof, int8_t joint_size, int8_t link_size, int8_t tool_size, bool* error = false)
  {
    dof_=dof;
    joint_size_ = joint_size;
    link_size_ = link_size;
    tool_size_ = tool_size;
  }

  void makeBase(char* base_name, Eigen::Vector3f base_position = Eigen::Vector3f::Zero(), Eigen::Matrix3f base_orientation = Eigen::Matrix3f::Identity(3,3), bool* error = false)
  {
    Base base_temp;
    base_[base_name] = base_temp;
    base_.at(base_name).init(base_position, base_orientation, error);
    if(!error)return;
    if(base_.size()>1)
    {
      // cout << "error : base size over" << endl;
      *error = true;
    }
  }

  void makeJoint(char* joint_name, int8_t actuator_id = -1, Eigen::Vector3f axis = Eigen::Vector3f::Zero(), bool* error = false)
  {
    Joint joint_temp;
    joint_[joint_name] = joint_temp;
    joint_.at(joint_name).init(actuator_id, axis, error);
    if(!error)return;
    if(joint_.size()>joint_size_)
    {
      // cout << "error : joint size over" << "(joint : " << joint_name << ")" << endl;
      *error = true;
    }
  }

  void makeTool(char* tool_name, int8_t actuator_id, Eigen::Vector3f axis, bool* error = false)
  {
    Tool tool_temp;
    tool_[tool_name] = tool_temp;
    tool_.at(tool_name).init(actuator_id, axis, error);
    if(!error)return;
    if(tool_.size()>tool_size_)
    {
      // cout << "error : tool size over"<< "(tool : " << tool_name << ")" << endl;
      *error = true;
    }
  }

  void makeLink(char* link_name, int8_t inner_control_point_size, bool* error = false)
  {
    Link link_temp;
    link_[link_name] = link_temp;
    link_.at(link_name).init(inner_control_point_size, error);
    Mass mass_temp;
    mass_.at(link_name) = mass_temp;
    if(!error)return;
    if(link_.size()>link_size_)
    {
    // cout << "error : link size over"<< "(link : " << link_name << ")" << endl;
      *error = true;
    }
  }

  void addControlPoint(char* link_name, char* point_name, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation, bool* error = false)
  {
    if(link_.find(link_name) != link_.end())
    {
      findControlPoint(point_name, error);
      if(!error)
      {
        link_.at(link_name).addControlPoint(point_name, relative_position, relative_orientation, error);
      }
      else
      {
        // cout << "error : Unknown control point"<< " (point name : " << point_name << ", link_name : " << link_name << ")" << endl;
      *error = true;
      }
    }
    else
    {
      // cout << "error : added control point to unknown link"<< " (point name : " << point_name << ", link_name : " << link_name << ")" << endl;
      *error = true;
    }
  }

  void setCenterOfMass(char* link_name, float mass, Eigen::Matrix3f initial_inertia_tensor, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation, bool* error = false)
  {
    if(link_.find(link_name) != link_.end())
    {
      link_.at(link_name).setCenterOfMass(mass, initial_inertia_tensor, relative_position, relative_orientation, error);
    }
    else
    {
      // cout << "error : set center of mass to unknown link"<< " (link_name : " << link_name << ")" << endl;
    *error = true;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Get function*///////////////////////////////
  int8_t getDOF(bool* error = false)
  {
    return dof_;
  }

  int8_t getJointSize(bool* error = false)
  {
    return joint_size_;
  }

  int8_t getLinkSize(bool* error = false)
  {
    return link_size_;
  }

  int8_t getToolSize(bool* error = false)
  {
    return tool_size_;
  }

  int8_t getControlPointSize(char* link_name, bool* error = false)
  {
    return link_[link_name].getControlPointSize();
  }

  char* getBaseName(bool* error = false)////////////////******
  {
    return base_.begin()->first;
  }

  vector<char*> getLinkControlPointNameList(char* link_name, bool* error = false)/////////////////////***********************
  {
    return link_.at(link_name).getLinkControlPointNameList(error);
  }

  vector<char*> getLinkNameList(char* point_name, bool* error = false)/////////////////////***********************
  {
    vector<char*> link_name_list;
    vector<char*> control_point_name_list;std::
    map<char*, Link>::iterator link_iterator;

    for(link_iterator = link_.begin(); link_iterator != link_.end(); link_iterator++)
    {
      control_point_name_list = getLinkControlPointNameList(link_iterator->first);
      for(int i = 0; i < control_point_name_list.size(); i++)
      {
        if(point_name == control_point_name_list.at(i))
        {
          link_name_list.push_back(link_iterator->first);
        }
      }
    }
    return link_name_list;
  }


  RelativePose getRelativePose(char* link_name, char* name, bool* error = false)
  {
    return link_.at(link_name).getRelativePose(name);
    
  }

  RelativePose getRelativePose(char* link_name, char* to_name, char* from_name, bool* error = false)
  {
    return link_.at(link_name).getRelativePose(to_name, from_name);
  }

  float getMass(char* link_name, bool* error = false)
  {
    return link_.at(link_name).getMass();
  }

  Eigen::Matrix3f getInitialInertiaTensor(char* link_name, bool* error = false)
  {
    return link_.at(link_name).getInitialInertiaTensor();
  } 

  RelativePose getRelativeCenterOfMassPose(char* link_name, bool* error = false)
  {
    return link_.at(link_name).getRelativeCenterOfMassPose();
  }

  RelativePose getRelativeCenterOfMassPose(char* link_name, char* from_name, bool* error = false)
  {
    return link_.at(link_name).getRelativeCenterOfMassPose(from_name);
  }

  Eigen::Vector3f getPosition(char* name, bool* error = false)
  {
    Pose reuslt;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      reuslt = control_point->getPose(error);
      return reuslt.position;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }
  
  Eigen::Matrix3f getOrientation(char* name, bool* error = false)
  {
    Pose reuslt;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      reuslt = control_point->getPose(error);
      return reuslt.orientation;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  Pose getPose(char* name, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      return control_point->getPose(error);
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  Eigen::Vector3f getLinearVelocity(char* name, bool* error = false)
  {
    DynamicPose result;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      result = control_point->getDynamicPose(error);
      return result.linear_velocity;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  Eigen::Vector3f getAngularVelocity(char* name, bool* error = false)
  {
    DynamicPose result;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      result = control_point->getDynamicPose(error);
      return result.angular_velocity;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  Eigen::Vector3f getLinearAcceleration(char* name, bool* error = false)
  {
    DynamicPose result;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      result = control_point->getDynamicPose(error);
      return result.linear_acceleration;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }
  
  Eigen::Vector3f getAngularAcceleration(char* name, bool* error = false)
  {
    DynamicPose result;
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      result = control_point->getDynamicPose(error);
      return result.angular_acceleration;
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  DynamicPose getDynamicPose(char* name, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      return control_point->getDynamicPose(error);
    }
    else
    {
      // cout << "error" << endl;
      // return;
    }
  }

  int8_t getJointActuatorId(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getActuatorId(error);
    }
    else
    {
      return -1;
    }
  }

  Eigen::Vector3f getJointAxis(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getAxis(error);
    }
    else
    {
      return Eigen::Vector3f::Zero();
    }
  }

  float getJointAngle(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getAngle(error);
    }
    else
    {
      return 0.0;
    }
  }

  float getJointAngularVelocity(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getAngularVelocity(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  float getJointAngularAcceleration(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getAngularAcceleration(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  JointState getJointJointState(char* joint_name, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      return joint_.at(joint_name).getJointState(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  int8_t getToolActuatorId(char* tool_name, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      return tool_.at(tool_name).getActuatorId(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  Eigen::Vector3f getToolAxis(char* tool_name, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      return tool_.at(tool_name).getAxis(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  bool getToolOnOff(char* tool_name, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      return tool_.at(tool_name).getOnOff(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }

  float getToolActuatorValue(char* tool_name, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      return tool_.at(tool_name).getActuatorValue(error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
      // return;
    }
  }
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////*Set function*///////////////////////////////
  void setBasePose(Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation, bool* error = false)
  {
    setPosition(getBaseName(), base_position, error);
    setOrientation(getBaseName(), base_orientation, error);
  }

  void setPosition(char* name, Eigen::Vector3f position, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setPosition(position, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }
  
  void setOrientation(char* name, Eigen::Matrix3f orientation, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setOrientation(orientation, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }
  
  void setPose(char* name, Pose pose, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setPose(pose, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }

  void setLinearVelocity(char* name, Eigen::Vector3f linear_velocity, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setLinearVelocity(linear_velocity, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }

  void setAngularVelocity(char* name, Eigen::Vector3f angular_velocity, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setAngularVelocity(angular_velocity, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }

  void setLinearAcceleration(char* name, Eigen::Vector3f linear_acceleration, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setLinearAcceleration(linear_acceleration, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }
  
  void setAngularAcceleration(char* name, Eigen::Vector3f angular_acceleration, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setAngularAcceleration(angular_acceleration, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }

  void setDynamicPose(char* name, DynamicPose dynamic_pose, bool* error = false)
  {
    ControlPoint* control_point;
    control_point = findControlPoint(name, error);
    if(!error)
    {
      control_point->setDynamicPose(dynamic_pose, error);
    }
    else
    {
      // cout << "error" << endl;
      return;
    }
  }

  void setJointAngle(char* joint_name, float angle, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      joint_.at(joint_name).setAngle(angle, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }

  void setJointAngularVelocity(char* joint_name, float velocity, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      joint_.at(joint_name).setAngularVelocity(velocity, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }

  void setJointAngularAcceleration(char* joint_name, float acceleration, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      joint_.at(joint_name).setAngularAcceleration(acceleration, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }

  void setJointState(char* joint_name, JointState state, bool* error = false)
  {
    if(joint_.find(joint_name) != joint_.end())
    {
      joint_.at(joint_name).setJointState(state, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }

  void setToolOnOff(char* tool_name, bool on_off, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      tool_.at(tool_name).setOnOff(on_off, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }


  void setToolActuatorValue(char* tool_name, float actuator_value, bool* error = false)
  {
    if(tool_.find(tool_name) != tool_.end())
    {
      tool_.at(tool_name).setActuatorValue(actuator_value, error);
    }
    else
    {
      // cout << "error" << endl;
    *error = true;
    }
  }
  ////////////////////////////////////////////////////////////////////////////


};

#endif

#endif // OMMANAGER_HPP_