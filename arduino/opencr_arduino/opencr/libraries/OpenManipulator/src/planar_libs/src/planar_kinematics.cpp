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

#include "../include/planar_libs/planar_kinematics.h"

using namespace robotis_manipulator;
using namespace planar_kinematics;

/*****************************************************************************
** Kinematics Solver 
*****************************************************************************/
void SolverUsingGeometry::setOption(const void *arg) {}

Eigen::MatrixXd SolverUsingGeometry::jacobian(Manipulator *manipulator, Name tool_name)
{
  return Eigen::MatrixXd::Identity(6, manipulator->getDOF());
}

void SolverUsingGeometry::solveForwardKinematics(Manipulator *manipulator) {}

bool SolverUsingGeometry::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}

/*****************************************************************************
** Private
*****************************************************************************/
bool SolverUsingGeometry::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value)
{
  const double link[3] = {0.120, 0.098, 0.0366};
  JointValue target_angle[7];
  std::vector<JointValue> target_angle_vector;
  double start_x[3], start_y[3];
  double temp_x[3], temp_y[3];
  double goal_x[3], goal_y[3];
  double diff_x[3], diff_y[3];
  Matrix3d goal_orientation;
  double target_pose_length[3];
  double alpha[3];
  double temp_diff[3];

  // Start pose for each set of two joints
  for (uint8_t i=0; i<3; i++)
  {
    start_x[i] = cos(PI*2.0/3.0*i)*(-0.1705);
    start_y[i] = sin(PI*2.0/3.0*i)*(-0.1705);
  }

  // Goal pose for each set of two joints after tool rotation
  for (uint8_t i=0; i<3; i++){
    temp_x[i] = target_pose.kinematic.position(0) + cos(PI*2.0/3.0*i)*(-link[2]);
    temp_y[i] = target_pose.kinematic.position(1) + sin(PI*2.0/3.0*i)*(-link[2]);
  }

  goal_orientation = target_pose.kinematic.orientation;
  if (goal_orientation(0,0) || goal_orientation(0,1) || goal_orientation(0,2)
   || goal_orientation(1,0) || goal_orientation(1,1) || goal_orientation(1,2)
   || goal_orientation(2,0) || goal_orientation(2,1) || goal_orientation(2,2))
  {
    goal_orientation(0,0) = 1;
    goal_orientation(1,1) = 1;
    goal_orientation(2,2) = 1;
  }
  for (uint8_t i=0; i<3; i++)
  {
    goal_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i];
    goal_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i];
    diff_x[i] = goal_x[i] - start_x[i];
    diff_y[i] = goal_y[i] - start_y[i];
    target_pose_length[i] = sqrt(diff_x[i]*diff_x[i] + diff_y[i]*diff_y[i]);
  }

  // Compute the Length of Position Difference and Target Angle
  for (uint8_t i=0; i<3; i++)
  {
    alpha[i] = acos((target_pose_length[i]*target_pose_length[i] + link[0]*link[0] - link[1]*link[1]) 
                            / (2*target_pose_length[i]*link[0]));
    temp_diff[i] = sin(-PI*2.0/3.0*i)*diff_x[i] + cos(-PI*2.0/3.0*i)*diff_y[i];
    target_angle[i].position = acos(-temp_diff[i] / target_pose_length[i]) - alpha[i] - PI/4.0;
  }

  // Set Joint Angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  for (uint8_t i=0; i<3; i++)
  {
    target_angle[i].position += PI/4.0; 
    target_angle[i+3].position = acos(-(temp_diff[i] - (-link[0]*cos(target_angle[i].position))) / link[1]) - target_angle[i].position - PI*7.0/12.0;
    target_angle[i+3].position = acos((-sin(PI*2.0/3.0*i)*diff_x[i] + cos(PI*2.0/3.0*i)*diff_y[i] 
                              + link[0]*cos(target_angle[i].position)) / -link[1]) - target_angle[i].position - PI*7.0/12.0;
  }

  target_angle_vector.push_back(target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(target_angle[5]);

  target_angle[3].position += PI*7.0/12.0;
  
  target_angle[6].position =  PI/2.0 - target_angle[0].position - target_angle[3].position + PI/3.0;

  target_angle_vector.push_back(target_angle[6]);
  
  *goal_joint_value = target_angle_vector;
  manipulator->setAllJointValue(target_angle_vector);

  return true;
}
