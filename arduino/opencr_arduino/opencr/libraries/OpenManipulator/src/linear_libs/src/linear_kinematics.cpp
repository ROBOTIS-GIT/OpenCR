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

#include "../include/linear_libs/linear_kinematics.h"

using namespace robotis_manipulator;
using namespace linear_kinematics;

/*****************************************************************************
** Kinematics Solver 
*****************************************************************************/
void SolverUsingGeometry::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingGeometry::jacobian(Manipulator *manipulator, Name tool_name)
{
  return Eigen::MatrixXd::Identity(6, manipulator->getDOF());
}

void SolverUsingGeometry::solveForwardKinematics(Manipulator *manipulator)
{
  forwardKinematicsSolverUsingGeometry(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingGeometry::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}

/*****************************************************************************
** Private
*****************************************************************************/
void SolverUsingGeometry::forwardKinematicsSolverUsingGeometry(Manipulator *manipulator, Name component_name)  
{
  // // Get my name, parent name and number of children 
  // Name my_name = component_name;
  // Name parent_name = manipulator->getComponentParentName(my_name);
  // int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  // // Define my & parent position and orientation 
  // Eigen::Vector3d parent_position_from_world;
  // Eigen::Matrix3d parent_orientation_from_world; 
  // Eigen::Vector3d my_position_from_world;
  // Eigen::Matrix3d my_orientation_from_world; 

 
  // // Get parent position and orientation 
  // // when parent is world
  // if (parent_name == manipulator->getWorldName())
  // {
  //   parent_position_from_world = manipulator->getWorldPosition();                 
  //   parent_orientation_from_world = manipulator->getWorldOrientation();
  // }
  // // when parent is not world
  // else
  // {
  //   parent_position_from_world = manipulator->getComponentPositionFromWorld(parent_name); 
  //   parent_orientation_from_world = manipulator->getComponentOrientationFromWorld(parent_name);
  // }

  // Calculate my position and orientation 
  double wheel_radius = 0.02818;
  double motor_angle[4];
  // std::vector<Name> child_name = manipulator->getComponentChildName(my_name);

  // for (int8_t index=0; index<2; index++) 
  // {
  motor_angle[0] = manipulator->getJointValue("joint1").position;
  motor_angle[1] = manipulator->getJointValue("joint2").position;
  motor_angle[2] = manipulator->getJointValue("joint3").position;
  motor_angle[3] = manipulator->getJointValue("tool").position;
  // }

  Eigen::Vector3d pos_joint1;
  Eigen::Vector3d pos_joint2;
  Eigen::Vector3d pos_joint3;
  Eigen::Vector3d pos_joint4;

  pos_joint1 << wheel_radius * motor_angle[0], 0                            , 0;
  pos_joint2 << wheel_radius * motor_angle[0], wheel_radius * motor_angle[1], 0;
  pos_joint3 << wheel_radius * motor_angle[0], wheel_radius * motor_angle[1], -0,01;
  pos_joint4 << wheel_radius * motor_angle[0], wheel_radius * motor_angle[1], wheel_radius * motor_angle[2];

  // Set my position and orientation 
  manipulator->setComponentPositionFromWorld("joint1", pos_joint1);
  manipulator->setComponentPositionFromWorld("joint2", pos_joint2);
  manipulator->setComponentPositionFromWorld("joint3", pos_joint3);
  manipulator->setComponentPositionFromWorld("tool", pos_joint4);

  Eigen::Vector3d pos_joint11, pos_joint22, pos_joint33;
  pos_joint11 = manipulator->getComponentPositionFromWorld("joint1");
  pos_joint22 = manipulator->getComponentPositionFromWorld("joint2");
  pos_joint33 = manipulator->getComponentPositionFromWorld("joint3");

  // Serial.print(pos_joint11(0));
  // Serial.print(pos_joint11(1));
  // Serial.print(pos_joint11(2));
  // Serial.print(pos_joint22(0));
  // Serial.print(pos_joint22(1));
  // Serial.print(pos_joint22(2));
  // Serial.print(pos_joint33(0));
  // Serial.print(pos_joint33(1));
  // Serial.println(pos_joint33(2));
  // Serial.flush();

}

bool SolverUsingGeometry::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  JointValue target_angle[3];       
  std::vector<JointValue> target_joint_value;
  double x_limit = 0.300;
  double y_limit = 0.300;
  double z_limit = 0.300;
  double wheel_radius = 0.02818;

  Manipulator _manipulator = *manipulator;

  // // Set x, y, z limits 
  // if (target_pose.position(0) >  x_limit) target_pose.position(0) = x_limit;
  // if (target_pose.position(0) < -x_limit) target_pose.position(0) = -x_limit;
  // if (target_pose.position(1) >  y_limit) target_pose.position(1) = y_limit;
  // if (target_pose.position(1) < -y_limit) target_pose.position(1) = -y_limit;
  // if (target_pose.position(2) >  z_limit) target_pose.position(1) = z_limit;
  // if (target_pose.position(2) <  0      ) target_pose.position(1) = 0;

  for (int8_t index=0; index<3; index++) 
  {
    // Calculate the target pose of each joint.
    target_angle[index].position = target_pose.kinematic.position(index) / wheel_radius;

    // Calculate the target pose of each joint.
    target_joint_value.push_back(target_angle[index]);
    // *goal_joint_value.push_back(target_angle[index]);    <-- can be like this???
  }
  *goal_joint_value = target_joint_value;

  return true;
}





