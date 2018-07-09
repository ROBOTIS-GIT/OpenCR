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

#include "../../include/open_manipulator/OMDebug.h"

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


Eigen::Vector3f makeEigenVector3(float v1, float v2, float v3);
Eigen::Matrix3f makeEigenMatrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);
Eigen::Matrix3f makeRotationMatrix(float roll, float pitch, float yaw);

class Joint
{
  private:
    int8_t dxl_id_;
    Eigen::Vector3f axis_;
    State joint_state_;
    Pose joint_pose_;

  public:
    /////////////////func///////////////////
    Joint();
    ~Joint();
    void init(int8_t dxl_id, Eigen::Vector3f axis);
    int8_t getId();
    Eigen::Vector3f getAxis();

    void setAngle(float angle);
    void setAngularVelocity(float angular_velocity);
    void setAngularAcceleration(float angular_acceleration);
    void setJointState(State joint_state);
    float getAngle();
    float getAngularVelocity();    
    float getAngularAcceleration();
    State getJointState();

    void setPosition(Eigen::Vector3f position);
    void setOrientation(Eigen::Matrix3f orientation);
    void setPose(Pose joint_pose);
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getOrientation();
    Pose getPose();
    ////////////////////////////////////////
};

class Link
{
  private:
    int8_t inner_joint_size_;
    Inertia inertia_;
    vector<InnerJoint> inner_joint_(1);

    Eigen::Vector3f center_position_;
  public:
    /////////////////func///////////////////
    Link();
    ~Link();
    void init(int8_t inner_joint_size);
    void setInnerJoint(int8_t joint_number, Eigen::Vector3f relative_position, Eigen::Matrix3f relative_orientation);

    int8_t getInnerJointSize();
    InnerJoint getInnerJointInformation(int8_t joint_number);
    Eigen::Vector3f getRelativeJointPosition(int8_t to, int8_t from);
    Eigen::Matrix3f getRelativeJointOrientation(int8_t to, int8_t from);
    Eigen::Vector3f getRelativeJointPosition(int8_t joint_number);
    Eigen::Matrix3f getRelativeJointOrientation(int8_t joint_number);
    
    int8_t findJoint(int8_t joint_number);

    void setInertia(Inertia inertia);
    void setMass(float mass);
    void setRelativeCenterPosition(Eigen::Vector3f relative_center_position);
    void setInertiaMoment(float inertia_moment);
    Inertia getInertia();
    float getMass();
    Eigen::Vector3f getRelativeCenterPosition();
    Eigen::Vector3f getRelativeCenterPosition(int8_t from);
    float getInertiaMoment();
    void setCenterPosition(Eigen::Vector3f center_position);
    Eigen::Vector3f getCenterPosition();
    ////////////////////////////////////////
};

class Base: public Link
{
  private:
    Pose relative_base_pose_;
    
    Pose base_pose_;
  public:
    /////////////////func///////////////////
    Base();
    ~Base();

    void setRelativeBasePostion(Eigen::Vector3f relative_base_position);
    void setRelativeBaseOrientation(Eigen::Matrix3f relative_base_orientation);
    void setRelativeBasePose(Pose relative_base_pose);

    void setPosition(Eigen::Vector3f base_position);
    void setOrientation(Eigen::Matrix3f base_orientation);
    void setPose(Pose base_pose);
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getOrientation();
    Pose getPose();
    Eigen::Vector3f getRelativeBaseJointPosition(int8_t to);
    Eigen::Matrix3f getRelativeBaseJointOrientation(int8_t to);
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
    Tool();
    ~Tool();
    void setToolType(int8_t tool_type);
    int8_t getToolType();

    void setRelativeToolPosition(Eigen::Vector3f relative_tool_position);
    void setRelativeToolOrientation(Eigen::Matrix3f relative_tool_orientation);
    void setRelativeToolPose(Pose relative_tool_pose);

    void setPosition(Eigen::Vector3f tool_position);
    void setOrientation(Eigen::Matrix3f tool_orientation);
    void setPose(Pose tool_pose);
    Eigen::Vector3f getPosition();
    Eigen::Matrix3f getOrientation();
    Pose getPose();
    Eigen::Vector3f getRelativeToolPosition(int8_t from);
    Eigen::Matrix3f getRelativeToolOrientation(int8_t from);
    ////////////////////////////////////////
};

template <int8_t TYPE, int8_t JOINT_SIZE, int8_t LINK_SIZE, int8_t TOOL_SIZE>
class Manipulator
{
  private:
    int8_t dof_;

  public:
    Base base_;
    Joint joint_[JOINT_SIZE];
    Link link_[LINK_SIZE];
    Tool tool_[TOOL_SIZE];

    /////////////////func///////////////////
    Manipulator(int8_t dof);
    ~Manipulator();

    void setDOF(int8_t dof);
    ////////////////////////////////////////
};

#endif // OMMANAGER_H_

