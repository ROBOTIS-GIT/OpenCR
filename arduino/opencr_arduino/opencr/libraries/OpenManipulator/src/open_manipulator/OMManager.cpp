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

Manipulator::Manipulator():
name_("UnknownManipulator"),
dof_(0)
{
  base_position_ = Eigen::Vector3f::Zero();
  base_orientation_ = Eigen::Matrix3f::Identity(3,3);
}
Manipulator::~Manipulator(){}

void Manipulator::Init(String name, int8_t dof, int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool)
{
  name_ = name;
    dof_=dof;
    number_of_joint_ = number_of_joint;
    number_of_link_ = number_of_link; 
    number_of_tool_ = number_of_tool;
}
void Manipulator::SetBasePosition(Eigen::Vector3f base_position)
{
  base_position_ = base_position;
}
void Manipulator::SetBaseOrientation(Eigen::Matrix3f base_orientation)
{
  base_orientation_ = base_orientation;
}
void Manipulator::SetDOF(int8_t dof)
{
    dof_=dof;
}

//////////////////////////////////////Joint////////////////////////////////////////////

Manipulator::Joint::Joint(): 
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
Manipulator::Joint::~Joint(){}
void Manipulator::Joint::Init(String name, int8_t number, int8_t dxl_id)
{
    name_ = name;
    number_ = number;
    dxl_id_ = dxl_id;
}

void Manipulator::Joint::SetAngle(float angle)
{
    angle_ = angle;
}

void Manipulator::Joint::SetVelocity(float velocity)
{
    velocity_ = velocity;
}

void Manipulator::Joint::SetAcceleration(float acceleration)
{
    acceleration_ = acceleration;
}

float Manipulator::Joint::GetAngle()
{
    return angle_;
}

float Manipulator::Joint::GetVelocity()
{
    return velocity_;
}

float Manipulator::Joint::GetAcceleration()
{
    return acceleration_;
}

void Manipulator::Joint::SetPosition(Eigen::Vector3f position)
{
    position_ = position;
}

void Manipulator::Joint::SetOrientation(Eigen::Matrix3f orientation)
{
    orientation_ = orientation;
}

Eigen::Vector3f Manipulator::Joint::GetPosition()
{
    return position_;
}

Eigen::Matrix3f Manipulator::Joint::GetOrientation()
{
    return orientation_;
}

Pose Manipulator::Joint::GetPose()
{
    Pose joint_pose;
    joint_pose.position = position_;
    joint_pose.orientation = orientation_;

    return joint_pose;
}

////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////Link/////////////////////////////////////////////

Manipulator::Link::Link(): 
    name_("UnknownLink"),
    mass_(0.0),
    inertia_moment_(0.0)
{
    center_position_ = Eigen::Vector3f::Zero();
}
Manipulator::Link::~Link(){}
void Manipulator::Link::Init(String name, int8_t number_of_joint_in_link)
{
    name_ = name;
    number_of_joint_in_link_ = number_of_joint_in_link;
}
void Manipulator::Link::Init(String name, int8_t number_of_joint_in_link, float mass, Eigen::Vector3f center_position)
{
    name_ = name;
    number_of_joint_in_link_ = number_of_joint_in_link;
    mass_ = mass;
    center_position_ = center_position;
}

float Manipulator::Link::GetInertiaMoment()
{
    return inertia_moment_;
}

Manipulator::Link::JointInLink::JointInLink():
number_(-1)
{
    relative_position_ = Eigen::Vector3f::Zero();
    relative_orientation_ = Eigen::Matrix3f::Identity(3,3);
}
Manipulator::Link::JointInLink::~JointInLink(){}
void Manipulator::Link::JointInLink::Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation)
{
    number_=number;
    relative_position_ = relative_position;
    relative_orientation_ = relative_orientation;
}
void Manipulator::Link::JointInLink::Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Vector3f axis)
{
    number_=number;
    relative_position_ = relative_position;
    relative_orientation_;
}
void Manipulator::Link::JointInLink::Init(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_rotation_matrix, Eigen::Vector3f axis)
{
    number_=number;
    relative_position_ = relative_position;
}

///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////Tool////////////////////////////////////////////

Manipulator::Tool::Tool():
tool_type_("null"),
number_(-1)
{
    position_from_final_joint_ = Eigen::Vector3f::Zero();
    orientation_from_final_joint_ = Eigen::Matrix3f::Identity(3,3);
    position_ = Eigen::Vector3f::Zero();
    orientation_ = Eigen::Matrix3f::Identity(3,3);
}
Manipulator::Tool::~Tool(){}
void Manipulator::Tool::Init(String tool_type, int8_t number, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint)
{
    tool_type_ = tool_type;
    number_ = number;
    position_from_final_joint_ = position_from_final_joint;
    orientation_from_final_joint_ = orientation_from_final_joint;
}
void Manipulator::Tool::SetPosition(Eigen::Vector3f position)
{
    position_ = position;
}
void Manipulator::Tool::SetOrientation(Eigen::Matrix3f orientation)
{
    orientation_ = orientation;
}
Eigen::Vector3f Manipulator::Tool::GetPosition()
{
    return position_;
}
Eigen::Matrix3f Manipulator::Tool::GetOrientation()
{
    return orientation_;
}
Pose Manipulator::Tool::GetPose()
{
    Pose tool_pose;
    tool_pose.position = position_;
    tool_pose.orientation = orientation_;

    return tool_pose;
}

///////////////////////////////////////////////////////////////////////////////////////






