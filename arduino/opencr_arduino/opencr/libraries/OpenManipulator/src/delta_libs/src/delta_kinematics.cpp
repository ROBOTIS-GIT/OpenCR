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

#include "../include/delta_libs/delta_kinematics.h"

using namespace robotis_manipulator; 
using namespace delta_kinematics;

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
bool SolverUsingGeometry::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  std::vector<JointValue> target_angle_vector;

  double temp_angle[3];
  double temp_angle2[3];
  JointValue target_angle[12];
  JointValue target_angle2[3];
  const double link[3] = {0.100, 0.224, 0.020};
  double start_x[3], start_y[3], start_z[3];
  double goal_x[3], goal_y[3], goal_z[3];
  double diff_x[3], diff_y[3], diff_z[3];
  double temp[3], temp2[3], temp3[3], temp4[3], temp5[3] , temp6[3];
  double a[3], b[3], c[3];
  double target_pose_length[3];
  double elbow_x[3], elbow_y[3], elbow_z[3];
  double diff_x2[3], diff_y2[3], diff_z2[3];

  // Start pose for each set of two joints
  for (int i=0; i<3; i++)
  {
    start_x[i] = cos(PI*2.0/3.0*i)*(-0.055);
    start_y[i] = sin(PI*2.0/3.0*i)*(-0.055);
    start_z[i] = 0.17875;
  }
  
  // Goal pose for each set of two joints
  for (int i=0; i<3; i++)
  {
    goal_x[i] = target_pose.kinematic.position(0) + cos(PI*2.0/3.0*i)*(-link[2]);
    goal_y[i] = target_pose.kinematic.position(1) + sin(PI*2.0/3.0*i)*(-link[2]);
    goal_z[i] = target_pose.kinematic.position(2);
  }

  // Pose difference for each set of two joints
  for (int i=0; i<3; i++)
  {
    diff_x[i] = goal_x[i] - start_x[i];
    diff_y[i] = goal_y[i] - start_y[i];
    diff_z[i] = goal_z[i] - start_z[i];
  }

  for (int i=0; i<3; i++)
  {
    temp[i] = diff_x[i]*cos(-PI*2.0/3.0*i) - diff_y[i]*sin(-PI*2.0/3.0*i);
    temp2[i] = diff_x[i]*sin(-PI*2.0/3.0*i) + diff_y[i]*cos(-PI*2.0/3.0*i);
    temp3[i] = sqrt(pow(link[1],2) - pow(temp2[i],2));
    temp4[i] = sqrt(pow(temp[i],2) + pow(diff_z[i],2));
    temp5[i] = pow(temp3[i],2) - pow(temp4[i],2) - pow(link[0],2);
    target_angle[i].position = asin(temp5[i] / (2*temp4[i]*link[0])) - acos(-diff_z[i]/temp4[i]);    
  }

  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  for (int i=0; i<3; i++){
    elbow_x[i] = start_x[i] + cos(PI*2.0/3.0*i)*link[0]*(-cos(target_angle[i].position));
    elbow_y[i] = start_y[i] + sin(PI*2.0/3.0*i)*link[0]*(-cos(target_angle[i].position));
    elbow_z[i] = start_z[i] + link[0]*sin(target_angle[i].position);
  }

  // Pose difference for each set of two joints
  for (int i=0; i<3; i++){
    diff_x2[i] = goal_x[i] - elbow_x[i];
    diff_y2[i] = goal_y[i] - elbow_y[i];
    diff_z2[i] = goal_z[i] - elbow_z[i];
  }

  for (int i=0; i<3; i++){
    temp6[i] = diff_x2[i]*sin(-PI*2.0/3.0*i) + diff_y2[i]*cos(-PI*2.0/3.0*i);
    target_angle[2*i+4].position = asin(temp6[i] / link[1]);
    target_angle[2*i+3].position = -PI + asin(-diff_z2[i] / cos(target_angle[2*i+4].position) / link[1]) - target_angle[i].position;
    target_angle[2*i+3].position += PI*127.0/180.0;
  }

  target_angle_vector.push_back(target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(target_angle[5]);
  target_angle_vector.push_back(target_angle[6]);
  target_angle_vector.push_back(target_angle[7]);
  target_angle_vector.push_back(target_angle[8]);

  target_angle[3].position -= PI*127.0/180.0;

  target_angle[9].position = -target_angle[4].position;
  target_angle[10].position = -PI - target_angle[0].position - target_angle[3].position;
  target_angle[10].position += PI*53.0/180.0;

  target_angle_vector.push_back(target_angle[9]);
  target_angle_vector.push_back(target_angle[10]);
  
  *goal_joint_value = target_angle_vector;
  manipulator->setAllJointValue(target_angle_vector);

  return true;
}
