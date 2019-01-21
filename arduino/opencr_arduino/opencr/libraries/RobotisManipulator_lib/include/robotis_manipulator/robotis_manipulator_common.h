/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef ROBOTIS_MANIPULATOR_COMMON_H
#define ROBOTIS_MANIPULATOR_COMMON_H

#include <unistd.h>
#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <WString.h>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
#endif
#include <math.h>
#include <vector>
#include <map>
#include "robotis_manipulator_math.h"
#include "robotis_manipulator_debug.h"

namespace ROBOTIS_MANIPULATOR
{

typedef STRING Name;

///////////////////// date set//////////////////////////////
//Pose struct
typedef struct _KinematicPose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
} KinematicPose;

typedef struct _Dynamicvector
{
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
} Dynamicvector;

typedef struct _DynamicPose
{
  Dynamicvector linear;
  Dynamicvector angular;
} DynamicPose;

//Inertia struct
typedef struct
{
  double mass;
  Eigen::Matrix3d inertia_tensor;
  Eigen::Vector3d center_of_mass;
} Inertia;

//Actuator Value Limit struct
typedef struct _Limit
{
  double maximum;
  double minimum;
} Limit;
////////////////////////////////////////////////////////////

/////////////////////////Time struct////////////////////////
typedef struct _Time
{
  double total_move_time;
  double present_time;
  double start_time;
}Time;
////////////////////////////////////////////////////////////

////////////////////Trajectory set//////////////////////////
//Trajectory struct
typedef enum _TrajectoryType
{
  NONE = 0,
  JOINT_TRAJECTORY,
  TASK_TRAJECTORY,
  CUSTOM_JOINT_TRAJECTORY,
  CUSTOM_TASK_TRAJECTORY
}TrajectoryType;

//Point struct
typedef struct _Point
{
  double position;
  double velocity;
  double acceleration;
  double effort;
}Point, Actuator, JointValue;

typedef std::vector<JointValue> JointWayPoint;

typedef struct _TaskWayPoint
{
  KinematicPose kinematic;
  DynamicPose dynamic;
}TaskWayPoint, PoseValue;

////////////////////////////////////////////////////////////

//////////////////////Componet data/////////////////////////
//Component Type
typedef enum _ComponentType
{
  PASSIVE_JOINT_COMPONENT = 0,
  ACTIVE_JOINT_COMPONENT,
  TOOL_COMPONENT
}ComponentType;

//Constant data
typedef struct _ChainigName
{
  Name parent;
  std::vector<Name> child;
} ChainigName;

typedef struct _Relative
{
  KinematicPose pose_from_parent;
  Inertia inertia;
} Relative;

typedef struct _JointConstant
{
  int8_t id;
  Eigen::Vector3d axis;
  double coefficient;       //actuator angle to joint angle
  Limit limit;
} JointConstant;

////////////////////////////////////////////////////////////

//////////////////////Componet set//////////////////////////
//World struct
typedef struct _World
{
  Name name;
  Name child;
  PoseValue pose;
} World;

//component struct
typedef struct _Component
{
  //constant
  ChainigName name;
  ComponentType component_type;
  Relative relative;
  JointConstant joint_constant;
  //variable
  PoseValue pose_from_world;
  JointValue joint_value;
  //Actuator
  Name actuator_name;
} Component;
////////////////////////////////////////////////////////////

/////////////////////Manipulator class//////////////////////
class Manipulator
{
private:
  int8_t dof_;
  World world_;
  std::map<Name, Component> component_;

public:
  Manipulator();
  ~Manipulator() {}

  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(3),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

  void addJoint(Name my_name,
                Name parent_name,
                Name child_name,
                Eigen::Vector3d relative_position,
                Eigen::Matrix3d relative_orientation,
                Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                int8_t joint_actuator_id = -1, double max_limit = M_PI, double min_limit = -M_PI,
                double coefficient = 1.0,
                double mass = 0.0,
                Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
                Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, double max_limit = M_PI, double min_limit = -M_PI,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(PoseValue world_pose);
  void setWorldKinematicPose(KinematicPose world_kinematic_pose);
  void setWorldPosition(Eigen::Vector3d world_position);
  void setWorldOrientation(Eigen::Matrix3d world_orientation);
  void setWorldDynamicPose(DynamicPose world_dynamic_pose);
  void setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity);
  void setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity);
  void setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration);
  void setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration);
  void setComponent(Name component_name, Component component);
  void setComponentActuatorName(Name component_name, Name actuator_name);
  void setComponentPoseFromWorld(Name component_name, PoseValue pose_to_world);
  void setComponentKinematicPoseFromWorld(Name component_name, KinematicPose pose_to_world);
  void setComponentPositionFromWorld(Name component_name, Eigen::Vector3d position_to_world);
  void setComponentOrientationFromWorld(Name component_name, Eigen::Matrix3d orientation_to_wolrd);
  void setComponentDynamicPoseFromWorld(Name component_name, DynamicPose dynamic_pose);

  void setJointPosition(Name name, double position);
  void setJointVelocity(Name name, double velocity);
  void setJointAcceleration(Name name, double acceleration);
  void setJointEffort(Name name, double effort);
  void setJointValue(Name name, JointValue joint_value);

  void setAllActiveJointPosition(std::vector<double> joint_position_vector);
  void setAllActiveJointValue(std::vector<JointValue> joint_value_vector);
  void setAllJointPosition(std::vector<double> joint_position_vector);
  void setAllJointValue(std::vector<JointValue> joint_value_vector);
  void setAllToolPosition(std::vector<double> tool_position_vector);
  void setAllToolValue(std::vector<JointValue> tool_value_vector);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();
  Name getWorldName();
  Name getWorldChildName();
  PoseValue getWorldPose();
  KinematicPose getWorldKinematicPose();
  Eigen::Vector3d getWorldPosition();
  Eigen::Matrix3d getWorldOrientation();
  DynamicPose getWorldDynamicPose();
  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentActuatorName(Name component_name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  PoseValue getComponentPoseFromWorld(Name name);
  KinematicPose getComponentKinematicPoseFromWorld(Name name);
  Eigen::Vector3d getComponentPositionFromWorld(Name name);
  Eigen::Matrix3d getComponentOrientationFromWorld(Name name);
  DynamicPose getComponentDynamicPoseFromWorld(Name name);
  KinematicPose getComponentRelativePoseFromParent(Name name);
  Eigen::Vector3d getComponentRelativePositionFromParent(Name name);
  Eigen::Matrix3d getComponentRelativeOrientationFromParent(Name name);

  int8_t getId(Name name);
  double getCoefficient(Name name);
  Eigen::Vector3d getAxis(Name name);
  double getJointPosition(Name name);
  double getJointVelocity(Name name);
  double getJointAcceleration(Name name);
  double getJointEffort(Name name);
  JointValue getJointValue(Name name);

  double getComponentMass(Name name);
  Eigen::Matrix3d getComponentInertiaTensor(Name name);
  Eigen::Vector3d getComponentCenterOfMass(Name name);

  std::vector<double> getAllJointPosition();
  std::vector<JointValue> getAllJointValue();
  std::vector<double> getAllActiveJointPosition();
  std::vector<JointValue> getAllActiveJointValue();
  std::vector<double> getAllToolPosition();
  std::vector<JointValue> getAllToolValue();

  std::vector<uint8_t> getAllJointID();
  std::vector<uint8_t> getAllActiveJointID();
  std::vector<Name> getAllToolComponentName();
  std::vector<Name> getAllActiveJointComponentName();

  ////////////////////////////////check function////////////////////////////////
  bool checkLimit(Name Component_name, double value);
  bool checkComponentType(Name component_name, ComponentType component_type);

  ///////////////////////////////Find function//////////////////////////////////
  Name findComponentNameFromId(int8_t id);
};

////////////////////////////////////////////////////////////
}

#endif // ROBOTIS_MANIPULATOR_COMMON_H
