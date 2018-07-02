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

#ifndef OMMANAGER_H_
#define OMMANAGER_H_

#include <unistd.h>
#include <WString.h>
#include <Eigen.h>       

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

typedef struct
{
  Eigen::Vector3f position;
  Eigen::Matrix3f orientation;
} Pose;

typedef struct
{
  float position;
  float velocity;
  float acceleration;
} State;


/////////////////////////////////////////Manipulator//////////////////////////////////////////

class Joint
{
  private:
    String name_;
    int8_t dxl_id_;
    int8_t number_;	

    float angle_;
    float velocity_;
    float acceleration_;

    Eigen::Vector3f position_;
    Eigen::Matrix3f orientation_;

  public:
    /////////////////func///////////////////
    Joint(): 
      name_("UnknownJoint"),
      dxl_id_(-1),
      number_(-1),
      angle_(0.0),
      velocity_(0.0),
      acceleration_(0.0)
    {
      position_ = Eigen::Vector3f::Zero();
      orientation_ = Eigen::Matrix3f::Identity(3,3);
    }
    ~Joint(){}
    void Init(String name, int8_t number, int8_t dxl_id)
    {
      name_ = name;
      number_ = number;
      dxl_id_ = dxl_id;
    }

    void SetAngle(float angle)
    {
      angle_ = angle;
    }

    void SetVelocity(float velocity)
    {
      velocity_ = velocity;
    }

    void SetAcceleration(float acceleration)
    {
      acceleration_ = acceleration;
    }

    float GetAngle()
    {
      return angle_;
    }

    float GetVelocity()
    {
      return velocity_;
    }

    float GetAcceleration()
    {
      return acceleration_;
    }

    void SetPosition(Eigen::Vector3f position)
    {
      position_ = position;
    }

    void SetOrientation(Eigen::Matrix3f orientation)
    {
      orientation_ = orientation;
    }

    Eigen::Vector3f GetPosition()
    {
      return position_;
    }

    Eigen::Matrix3f GetOrientation()
    {
      return orientation_;
    }

    Pose GetPose()
    {
      Pose joint_pose;
      joint_pose.position = position_;
      joint_pose.orientation = orientation_;
      return joint_pose;
    }
    ////////////////////////////////////////
};

class Link
{
  private:
    String name_;
    float mass_;			
    float inertia_moment_;
    int8_t number_of_joint_in_link_;

    Eigen::Vector3f center_position_;

  public:
    /////////////////func///////////////////
    Link(): 
    name_("UnknownLink"),
    mass_(0.0),
    inertia_moment_(0.0)
    {
      center_position_ = Eigen::Vector3f::Zero();
    }
    ~Link(){}
    void Init(String name, int8_t number_of_joint_in_link)
    {
      name_ = name;
      number_of_joint_in_link_ = number_of_joint_in_link;
      //JointInLink jointinlink[number_of_joint_in_link];
    }
    void Init(String name, int8_t number_of_joint_in_link, float mass, Eigen::Vector3f center_position)
    {
      name_ = name;
      number_of_joint_in_link_ = number_of_joint_in_link;
      mass_ = mass;
      center_position_ = center_position;
    }

    float GetInertiaMoment()
    {
      return inertia_moment_;
    }
    ////////////////////////////////////////
};

class JointInLink
{
  private:
  int8_t number_;
            
  Eigen::Vector3f relative_position_;
  Eigen::Matrix3f relative_orientation_;
  public:
    /////////////////func///////////////////
    JointInLink():
    number_(-1)
    {
      relative_position_ = Eigen::Vector3f::Zero();
      relative_orientation_ = Eigen::Matrix3f::Identity(3,3);
    }
    ~JointInLink(){}
    void Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
    {
      number_=number;
      relative_position_ = relative_position;
      relative_orientation_ = relative_orientation;
    }
    void Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Vector3f axis)
    {
      number_=number;
      relative_position_ = relative_position;
      relative_orientation_;
    }
    void Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_rotation_matrix, Eigen::Vector3f axis)
    {
      number_=number;
      relative_position_ = relative_position;
    }
    ////////////////////////////////////////
};

class Tool
{
  private:
    String tool_type_;
    int8_t number_;

    Eigen::Vector3f position_from_final_joint_;
    Eigen::Matrix3f orientation_from_final_joint_;

    Eigen::Vector3f position_;
    Eigen::Matrix3f orientation_;

  public:
    Tool():
    tool_type_("null"),
    number_(-1)
    {
      position_from_final_joint_ = Eigen::Vector3f::Zero();
      orientation_from_final_joint_ = Eigen::Matrix3f::Identity(3,3);
      position_ = Eigen::Vector3f::Zero();
      orientation_ = Eigen::Matrix3f::Identity(3,3);
    }
    ~Tool(){}
    void Init(String tool_type, int8_t number, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint)
    {
      tool_type_ = tool_type;
      number_ = number;
      position_from_final_joint_ = position_from_final_joint;
      orientation_from_final_joint_ = orientation_from_final_joint;
    }
    void SetPosition(Eigen::Vector3f position)
    {
      position_ = position;
    }
    void SetOrientation(Eigen::Matrix3f orientation)
    {
      orientation_ = orientation;
    }
    Eigen::Vector3f GetPosition()
    {
      return position_;
    }
    Eigen::Matrix3f GetOrientation()
    {
      return orientation_;
    }
    Pose GetPose()
    {
      Pose tool_pose;
      tool_pose.position = position_;
      tool_pose.orientation = orientation_;
      return tool_pose;
    }
};

//template <int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool>
class Manipulator
{
  private:
    int8_t dof_;
    String name_;
    Eigen::Vector3f base_position_;
    Eigen::Matrix3f base_orientation_;
    int8_t number_of_joint_;
    int8_t number_of_link_;
    int8_t number_of_tool_;
    

  public:
    /////////////////func///////////////////
    Joint joint_[3];
    Link link_[2];
    Tool tool_[1];
    Manipulator(String name, int8_t dof):
    number_of_joint_(0),
    number_of_link_(0),
    number_of_tool_(0)
    {
      name_ = name;
      dof_ = dof;
      base_position_ = Eigen::Vector3f::Zero();
      base_orientation_ = Eigen::Matrix3f::Identity(3,3);
    }
    ~Manipulator(){}
    void Load(Joint* joint, Link* link, Tool* tool)
    {
      joint_ < joint;
      link_ < link;
      tool_ < tool;
    }
    void SetBasePosition(Eigen::Vector3f base_position)
    {
      base_position_ = base_position;
    }
    void SetBaseOrientation(Eigen::Matrix3f base_orientation)
    {
      base_orientation_ = base_orientation;
    }
    void SetDOF(int8_t dof)
    {
      dof_=dof;
    }
    ////////////////////////////////////////
};   

#endif // OMMANAGER_H_

