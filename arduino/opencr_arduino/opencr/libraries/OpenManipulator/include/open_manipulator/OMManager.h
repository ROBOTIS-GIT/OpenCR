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

class Manipulator;
class Base;
class Joint;
class Link;
class Tool;

template <int8_t number_of_joint_, int8_t number_of_link_, int8_t number_of_tool_>
class Manipulator
{
  private:
    int8_t dof_;
    String name_;

  public:
    Base base;
    Joint joint[number_of_joint_];
    Link link[number_of_link_];
    Tool tool[number_of_tool_];

    /////////////////func///////////////////
    Manipulator(String name, int8_t dof);
    Manipulator(int8_t dof);
    ~Manipulator();
    void setDOF(int8_t dof);
    ////////////////////////////////////////
};   


class Base
{
  private:
    int8_t counter_;
    int8_t number_of_base_joint_;
    float mass_;			
    float inertia_moment_;
    Eigen::Vector3f center_position_;
    Eigen::Vector3f base_position_;
    Eigen::Matrix3f base_orientation_;

    vector<JointInLink> base_joint_(1);

  public:
    /////////////////func///////////////////
    Base();
    ~Base();
    void init(int8_t number_of_base_joint);
    void init(int8_t number_of_base_joint, float mass, Eigen::Vector3f center_position);
    void init(int8_t number_of_base_joint, float mass, float inertia_moment, Eigen::Vector3f center_position);
    void setBaseJoint(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    void setBaseJoint(int8_t base_joint_number, int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    JointInLink getJointInformation(int8_t base_joint_number);
    int8_t getTheNumberOfJoint();
    float getMass();
    void setMass(float mass);
    float getInertiaMoment();
    void setInertiaMoment(float inertia_moment);
    int8_t findJoint(int8_t joint_number);
    Eigen::Vector3f getCenterPosition(); 
    Eigen::Vector3f getBaseJointPosition(int8_t to);
    Eigen::Vector3f getBaseJointOrientation(int8_t to);
    void setBasePosition(Eigen::Vector3f base_position);
    void setBaseOrientation(Eigen::Matrix3f base_orientation);
    void setBasePose(Pose base_pose);
    Eigen::Vector3f getBasePosition();
    Eigen::Matrix3f getBaseOrientation();
    Pose getBasePose();
    ////////////////////////////////////////
};

class Joint
{
  private:
    int8_t dxl_id_;

    float angle_;
    float velocity_;
    float acceleration_;
    Eigen::Vector3f axis_;

    Eigen::Vector3f position_;
    Eigen::Matrix3f orientation_;

  public:
    /////////////////func///////////////////
    Joint();
    ~Joint();
    void init(int8_t dxl_id, Eigen::Vector3f axis);
    int8_t getId();
    Eigen::Vector3f getAxis();
    void setAngle(float angle);
    float getAngle();
    void setVelocity(float velocity);
    float getVelocity();    
    void setAcceleration(float acceleration);
    float getAcceleration();
    void setPosition(Eigen::Vector3f position);
    Eigen::Vector3f getPosition();
    void setOrientation(Eigen::Matrix3f orientation);
    Eigen::Matrix3f getOrientation();
    void setPose(Pose joint_pose);
    Pose getPose();
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
    void init(int8_t number_of_joint_in_link);
    void init(int8_t number_of_joint_in_link, float mass, Eigen::Vector3f center_position);
    void init(int8_t number_of_joint_in_link, float mass, float inertia_moment, Eigen::Vector3f center_position);
    void setJointInLink(int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    void setJointInLink(int8_t joint_in_link_number, int8_t number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    JointInLink getJointInformation(int8_t joint_in_link_number);
    int8_t getTheNumberOfJoint();
    float getMass();
    void setMass(float mass);
    float getInertiaMoment();
    void setInertiaMoment(float inertia_moment);
    int8_t findJoint(int8_t joint_number);
    Eigen::Vector3f getCenterPosition();
    Eigen::Vector3f getCenterPosition(int8_t from);    
    Eigen::Vector3f getRelativeJointPosition(int8_t to, int8_t from);
    Eigen::Vector3f getRelativeJointOrientation(int8_t to, int8_t from);
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
    void init(String tool_type, Eigen::Vector3f position_from_final_joint, Eigen::Matrix3f orientation_from_final_joint);
    Eigen::Vector3f getRelativePosition();
    Eigen::Matrix3f getRlativeOrientation();
    Pose getRelativePose();
    void setPosition(Eigen::Vector3f position);
    void setOrientation(Eigen::Matrix3f orientation);
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getOrientation();
    Pose getPose();
    ////////////////////////////////////////
};


#endif // OMMANAGER_H_

