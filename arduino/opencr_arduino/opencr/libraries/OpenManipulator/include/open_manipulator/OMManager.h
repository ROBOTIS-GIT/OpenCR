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
  Eigen::Vector3f center_position;
  float moment;
} Inertia;


class Manipulator;
class Link;
class Base;
class Joint;
class Tool;

template <int8_t type_, int8_t joint_size_, int8_t link_size_, int8_t tool_size>
class Manipulator
{
  private:
    int8_t dof_;

  public:
    Base base_;
    Joint joint_[joint_size_];
    Link link_[link_size_];
    Tool tool_[tool_size];

    /////////////////func///////////////////
    Manipulator(int8_t dof);
    ~Manipulator();
    void setDOF(int8_t dof);
    ////////////////////////////////////////
};

class Link
{
  private:
    int8_t inner_joint_size_;
    Inertia inertia_;
    vector<InnerJoint> inner_joint_(1);
  public:
    /////////////////func///////////////////
    Link();
    ~Link();
    void init(int8_t inner_joint_size);

    void setInertia(Inertia inertia);
    void setMass(float mass);
    void setCenterPosition(Eigen::Vector3f center_position);
    void setInertiaMoment(float inertia_moment);
    Inertia getInertia();
    float getMass();
    Eigen::Vector3f getCenterPosition();
    Eigen::Vector3f getCenterPosition(int8_t from);
    float getInertiaMoment();

    int8_t getInnerJointSize();
    void setInnerJoint(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);
    InnerJoint getInnerJointInformation(int8_t joint_number);
    Eigen::Vector3f getRelativeJointPosition(int8_t to, int8_t from);
    Eigen::Vector3f getRelativeJointOrientation(int8_t to, int8_t from);
    
    int8_t findJoint(int8_t joint_number);
    ////////////////////////////////////////
};

class Base: public Link
{
  private:
    Pose base_pose_;
  public:
    /////////////////func///////////////////
    Base();
    ~Base();
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
    State joint_state_;
    Eigen::Vector3f axis_;
    Pose joint_pose_;

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


class Tool
{
  private:
    int8_t tool_type_;
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

