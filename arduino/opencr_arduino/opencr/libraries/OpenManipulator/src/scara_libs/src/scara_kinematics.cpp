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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/scara_libs/scara_kinematics.h"

using namespace robotis_manipulator;
using namespace scara_kinematics;

/*****************************************************************************
** Kinematics Solver 
*****************************************************************************/
void SolverUsingCRAndGeometry::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingCRAndGeometry::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void SolverUsingCRAndGeometry::solveForwardKinematics(Manipulator *manipulator)
{
  forwardKinematicsSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndGeometry::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}

/*****************************************************************************
** Private
*****************************************************************************/
void SolverUsingCRAndGeometry::forwardKinematicsSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name));
  //linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardKinematicsSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingCRAndGeometry::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value)
{
  const double link[3] = {0.067, 0.067, 0.107};
  JointValue target_angle[3];
  std::vector<JointValue> target_angle_vector;

  // Compute the length from Joint1 to the end effector
  double temp_target_pose[2];
  double target_pose_length;
  temp_target_pose[0] = target_pose.kinematic.position(0) + 0.241;
  temp_target_pose[1] = target_pose.kinematic.position(1);
  target_pose_length = sqrt((temp_target_pose[0])*temp_target_pose[0] + temp_target_pose[1]*temp_target_pose[1]);

  // Compute the length of Position Difference and Target Angle
  double error=1000.0; // random large initial value

  for (uint16_t count=0; count<=900; count++){
    double theta=(double)count/10*DEG2RAD;

    // Assume theta = target_angle[1] = target_angle[2]
    double alpha = acos((link[1]*link[1]+link[2]*link[2]-link[0]*link[0]-target_pose_length*target_pose_length+2*link[1]*link[2]*cos(theta))
                  / (-2*target_pose_length*link[0]));
    double beta = acos((link[0]*link[0]+link[1]*link[1]-link[2]*link[2]-target_pose_length*target_pose_length+2*link[0]*link[1]*cos(theta))
                  / (-2*target_pose_length*link[2]));
    double temp_error = abs(alpha + beta - 2*theta);

    if (temp_error < error){
      target_angle[0].position = PI/2 -acos(temp_target_pose[1]/target_pose_length) - alpha;
      target_angle[1].position = theta;
      target_angle[2].position = theta;
      error = temp_error;
    }
  }

  // Set joint angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  *goal_joint_value = target_angle_vector; 
   
  return true;
}
