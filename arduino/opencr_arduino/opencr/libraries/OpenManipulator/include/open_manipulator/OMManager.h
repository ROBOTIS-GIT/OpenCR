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
#include <vector>       

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

using namespace std;

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

typedef struct
{
  int8_t number;
  Eigen::Vector3f relative_position;
  Eigen::Matrix3f relative_orientation;
} JointInLink;

class Joint
{
  private:
    int8_t dxl_id_;

    float angle_;
    float velocity_;
    float acceleration_;

    Eigen::Vector3f position_;
    Eigen::Matrix3f orientation_;

  public:
    /////////////////func///////////////////
    Joint();
    ~Joint();
    void SetId(int8_t dxl_id);
    int8_t GetId();
    void SetAngle(float angle);
    float GetAngle();
    void SetVelocity(float velocity);
    float GetVelocity();    
    void SetAcceleration(float acceleration);
    float GetAcceleration();
    void SetPosition(Eigen::Vector3f position);
    Eigen::Vector3f GetPosition();
    void SetOrientation(Eigen::Matrix3f orientation);
    Eigen::Matrix3f GetOrientation();
    void SetPose(Pose joint_pose);
    Pose GetPose();
    ////////////////////////////////////////
};

class Link
{
  private:

    int8_t counter_;
    int8_t number_of_joint_in_link_;
    float mass_;			
    float inertia_moment_;
    Eigen::Vector3f center_position_;

    vector<JointInLink> jointinlink_(1);

  public:
    /////////////////func///////////////////
    Link();
    ~Link();
    void Init(int8_t number_of_joint_in_link);
    void Init(int8_t number_of_joint_in_link, float mass, Eigen::Vector3f center_position);
    void Init(int8_t number_of_joint_in_link, float mass, float inertia_moment, Eigen::Vector3f center_position);
    void SetJointInLink(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    void SetJointInLink(int8_t joint_in_link_number, int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    JointInLink GetJointInformation(int8_t joint_in_link_number);
    int8_t GetTheNumberOfJoint();
    float GetMass();
    void SetMass(float mass);
    float GetInertiaMoment();
    void SetInertiaMoment(float inertia_moment);
    int8_t FindJoint(int8_t joint_number);
    Eigen::Vector3f GetCenterPosition();
    Eigen::Vector3f GetCenterPosition(int8_t from);    
    Eigen::Vector3f GetRelativeJointPosition(int8_t to, int8_t from);
    Eigen::Vector3f GetRelativeJointOrientation(int8_t to, int8_t from);
    ////////////////////////////////////////
};

class Tool
{
  private:
    String tool_type_;
    Eigen::Vector3f position_from_final_joint_;
    Eigen::Matrix3f orientation_from_final_joint_;

    Eigen::Vector3f position_;
    Eigen::Matrix3f orientation_;

  public:
    /////////////////func///////////////////
    Tool();
    ~Tool();
    void Init(String tool_type, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint);
    Eigen::Vector3f GetRelativePosition();
    Eigen::Matrix3f GetRlativeOrientation();
    Pose GetRelativePose();
    void SetPosition(Eigen::Vector3f position);
    void SetOrientation(Eigen::Matrix3f orientation);
    Eigen::Vector3f GetPosition();
    Eigen::Matrix3f GetOrientation();
    Pose GetPose();
    ////////////////////////////////////////
};

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
    vector<Joint> joint(1);
    vector<Link> link(1);
    vector<Tool> tool(1);

    /////////////////func///////////////////
    Manipulator(String name, int8_t dof);
    Manipulator(int8_t dof);
    ~Manipulator();
    void Init(int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool);
    void SetDOF(int8_t dof);
    void SetBasePosition(Eigen::Vector3f base_position);
    void SetBaseOrientation(Eigen::Matrix3f base_orientation);
    void SetBasePose(Pose base_pose);
    Eigen::Vector3f GetBasePosition();
    Eigen::Matrix3f GetBaseOrientation();
    Pose GetBasePose();
    ////////////////////////////////////////
};   

#endif // OMMANAGER_H_

