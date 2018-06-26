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


#include "../../include/OMManager.h"

Manipulator::Manipulator(){}
Manipulator::~Manipulator(){}
void Manipulator::Init(int8_t dof, int8_t number_of_joint, int8_t number_of_link)
{
    DOF_=dof;
    Manipulator::Joint OMJoint[number_of_joint];
    Manipulator::Link OMLink[number_of_link];
    Manipulator::Tool OMTool[number_of_link];
}

void Manipulator::Init(int8_t dof, int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool);

void Manipulator::SetDOF(int8_t dof)
{


}



Manipulator::Joint::Joint(){}
Manipulator::Joint::~Joint(){}
void Manipulator::Joint::Init(String name, int8_t number, int8_t dxl_id)
{

}

void Manipulator::Joint::SetAngle(float angle)
{

}

void Manipulator::Joint::SetVelocity(float velocity)
{

}

void Manipulator::Joint::SetAcceleration(float acceleration)
{

}

float Manipulator::Joint::GetAngle()
{

}

float Manipulator::Joint::GetVelocity()
{

}

float Manipulator::Joint::GetAcceleration()
{

}

void Manipulator::Joint::SetPosition(Eigen::Vector3f position)
{

}

void Manipulator::Joint::SetOrientation(Eigen::Matrix3f orientation)
{

}

Eigen::Vector3f Manipulator::Joint::GetPosition()
{

}

Eigen::Matrix3f Manipulator::Joint::GetOrientation()
{

}

Pose GetPose()
{

}



Manipulator::Link::Link(){}
Manipulator::Link::~Link(){}
void Manipulator::Link::Init(String name, float mass, Eigen::Vector3f center_position, int8_t number_of_joint_in_link)
{

}
float Manipulator::Link::InertiaMoment()
{

}



JointInLink();
~JointInLink();
void Manipulator::Link::JointInLink::Init(int8_t joint_number, String type, Eigen::Vector3f relative_position, Eigen::Vector3f axis)
{

}
void Manipulator::Link::JointInLink::Init(int8_t joint_number, String type, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_axis_matrix, Eigen::Vector3f axis)
{

}








