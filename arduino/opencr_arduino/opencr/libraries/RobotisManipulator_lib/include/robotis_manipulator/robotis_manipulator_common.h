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
typedef struct _Pose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
} Pose;

typedef struct _Dynamicvector
{
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
} Dynamicvector;

typedef struct _Dynamicpose
{
  Dynamicvector linear;
  Dynamicvector angular;
} Dynamicpose;

//Inertia struct
typedef struct
{
  double mass;
  Eigen::Matrix3d inertia_tensor;
  Eigen::Vector3d center_of_mass;
} Inertia;

//Actuator Value Limit struct
typedef struct _AcutuatorLimit
{
  double maximum;
  double minimum;
} Limit;
////////////////////////////////////////////////////////////

/////////////////////////Time struct////////////////////////
typedef struct _Time
{
  double total_move_time;
  double control_loop_time;
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
  DRAWING_TRAJECTORY
}TrajectoryType;

typedef enum _WayPointType
{
  JOINT_WAY_POINT = 0,
  TASK_WAY_POINT
}WayPointType;

//Point struct
typedef struct _Point
{
  double value;
  double velocity;
  double acceleration;
  double effort;
} Actuator, WayPoint;
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
  Pose pose_from_parent;
  Inertia inertia;
} Relative;

typedef struct _ActuatorConstant
{
  int8_t id;
  Eigen::Vector3d axis;
  double coefficient;       //actuator angle to joint angle
  Limit limit;
} ActuatorConstant;

//Variable data
typedef struct _ActuatorVariable
{
  double value;
  double velocity;
  double acceleration;
  double effort;
} ActuatorVariable;

typedef struct _PoseVariable
{
  Pose pose;
  Dynamicpose dynamic_pose;
} PoseVariable;

////////////////////////////////////////////////////////////

//////////////////////Componet set//////////////////////////
//World struct
typedef struct _World
{
  Name name;
  Name child;
  Pose pose;
  Dynamicpose dynamic_pose;
} World;

//component struct
typedef struct _Component
{
  //constant
  ChainigName name;
  ComponentType component_type;
  Relative relative;
  ActuatorConstant actuator_constant;
  //variable
  PoseVariable from_world;
  ActuatorVariable actuator_variable;
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
  void setWorldPose(Pose world_pose);
  void setWorldPosition(Eigen::Vector3d world_position);
  void setWorldOrientation(Eigen::Matrix3d world_orientation);
  void setWorldDynamicPose(Dynamicpose world_dynamic_pose);
  void setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity);
  void setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity);
  void setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration);
  void setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration);
  void setComponent(Name component_name, Component component);
  void setComponentActuatorName(Name component_name, Name actuator_name);
  void setComponentPoseFromWorld(Name name, Pose pose_to_world);
  void setComponentPositionFromWorld(Name name, Eigen::Vector3d position_to_world);
  void setComponentOrientationFromWorld(Name name, Eigen::Matrix3d orientation_to_wolrd);
  void setComponentDynamicPoseFromWorld(Name name, Dynamicpose dynamic_pose);

  void setValue(Name name, double value);
  void setVelocity(Name name, double velocity);
  void setAcceleration(Name name, double acceleration);
  void setEffort(Name name, double effort);
  void setJointValue(Name name, WayPoint way_point);

  void setAllActiveJointValue(std::vector<double> joint_value_vector);
  void setAllActiveJointValue(std::vector<WayPoint> joint_way_point_vector);
  void setAllJointValue(std::vector<double> joint_value_vector);
  void setAllJointValue(std::vector<WayPoint> joint_way_point_vector);

  void setAllToolValue(std::vector<double> tool_value_vector);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();
  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Eigen::Vector3d getWorldPosition();
  Eigen::Matrix3d getWorldOrientation();
  Dynamicpose getWorldDynamicPose();
  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentActuatorName(Name component_name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseFromWorld(Name name);
  Eigen::Vector3d getComponentPositionFromWorld(Name name);
  Eigen::Matrix3d getComponentOrientationFromWorld(Name name);
  Dynamicpose getComponentDynamicPoseFromWorld(Name name);
  Pose getComponentRelativePoseFromParent(Name name);
  Eigen::Vector3d getComponentRelativePositionFromParent(Name name);
  Eigen::Matrix3d getComponentRelativeOrientationFromParent(Name name);

  int8_t getId(Name name);
  double getCoefficient(Name name);
  Eigen::Vector3d getAxis(Name name);
  double getValue(Name name);
  double getVelocity(Name name);
  double getAcceleration(Name name);
  double getEffort(Name name);

  double getComponentMass(Name name);
  Eigen::Matrix3d getComponentInertiaTensor(Name name);
  Eigen::Vector3d getComponentCenterOfMass(Name name);

  std::vector<double> getAllJointValue();
  std::vector<WayPoint> getAllJointWayPoint();
  std::vector<double> getAllActiveJointValue();
  std::vector<WayPoint> getAllActiveJointWayPoint();
//  void getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector, std::vector<double> *joint_effort_vector=NULL);

  std::vector<double> getAllToolValue();

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
