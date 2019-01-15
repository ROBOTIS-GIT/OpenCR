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

#include "../include/open_manipulator_libs/Planar_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR;
using namespace PLANAR_KINEMATICS;

void Planar::setOption(const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  if(get_arg_[0] =="inverse_solver")
  {
    STRING inverse_kinematics_solver_option = get_arg_[1];
    inverse_kinematics_solver_option_ = inverse_kinematics_solver_option;
  }
}

void Planar::updatePassiveJointValue(Manipulator *manipulator)
{}

Eigen::MatrixXd Planar::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  return jacobian;
}

void Planar::forwardKinematics(Manipulator *manipulator)  
{
  forwardKinematicsSolverUsingGeometry(manipulator, manipulator->getWorldChildName());
}

bool Planar::inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double> *goal_joint_value)
{
  return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}

/*-----------------------------------------------------------------------------------------------*/

void Planar::forwardKinematicsSolverUsingGeometry(Manipulator *manipulator, Name component_name)
{
}

bool Planar::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double> *goal_joint_value)
{
  double link[3] = {0.120, 0.098, 0.0366};
  double target_angle[3];
  std::vector<double> target_angle_vector;

  // Start pose for each set of two joints
  double start_x[3], start_y[3];    
  for (uint8_t i=0; i<3; i++){
    start_x[i] = cos(PI*2.0/3.0*i)*(-0.1705);
    start_y[i] = sin(PI*2.0/3.0*i)*(-0.1705);
  }

  // Goal pose for each set of two joints after tool rotation
  double temp_x[3];       
  double temp_y[3];
  double goal_x[3];     
  double goal_y[3];
  double diff_x[3];       
  double diff_y[3];
  Matrix3d goal_orientation;
  double target_pose_length[3];

  for (int i=0; i<3; i++){
    temp_x[i] = target_pose.position(0) + cos(PI*2.0f/3.0f*i)*(-link[2]);
    temp_y[i] = target_pose.position(1) + sin(PI*2.0f/3.0f*i)*(-link[2]);
  }

  ////// This part should be removed later...after adding FK
  goal_orientation = target_pose.orientation;
  if (goal_orientation(0,0) || goal_orientation(0,1) || goal_orientation(0,2)
   || goal_orientation(1,0) || goal_orientation(1,1) || goal_orientation(1,2)
   || goal_orientation(2,0) || goal_orientation(2,1) || goal_orientation(2,2))
  {
    goal_orientation(0,0) = 1;
    goal_orientation(1,1) = 1;
    goal_orientation(2,2) = 1;
  }
  for (int i=0; i<3; i++)
  {
    goal_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i];
    goal_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i];
    diff_x[i] = goal_x[i] - start_x[i];
    diff_y[i] = goal_y[i] - start_y[i];
    target_pose_length[i] = sqrt(diff_x[i]*diff_x[i] + diff_y[i]*diff_y[i]);
  }

  // Compute the lngth of position difference and target angle
  double alpha[3];
  double temp_diff[3];      

  for (int i=0; i<3; i++){
    alpha[i] = acos((target_pose_length[i]*target_pose_length[i] + link[0]*link[0] - link[1]*link[1]) 
                            / (2*target_pose_length[i]*link[0]));
    temp_diff[i] = sin(-PI*2.0/3.0*i)*diff_x[i] + cos(-PI*2.0/3.0*i)*diff_y[i];
    target_angle[i] = acos(-temp_diff[i] / target_pose_length[i]) - alpha[i] - PI/4.0f;
  }

  // Set joint angle 
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  *goal_joint_value = target_angle_vector;

  return true;
}
