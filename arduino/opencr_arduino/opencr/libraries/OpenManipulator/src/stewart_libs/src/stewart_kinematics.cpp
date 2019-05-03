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

#include "../include/stewart_libs/stewart_kinematics.h"

using namespace robotis_manipulator;
using namespace stewart_kinematics;

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

  double temp_angle[6];
  double temp_angle2[6];
  JointValue target_angle[21];
  double link[2] = {0.026, 0.1227};
  double start_x[6], start_y[6], start_z[6],
         temp_x[6], temp_y[6], temp_z[6],
         target_x[6], target_y[6], target_z[6],
         diff_x[6], diff_y[6], diff_z[6];
  double temp[6], temp2[6];
  double target_pose_length[6];
  Matrix3d goal_orientation;
  double elbow_x[6], elbow_y[6], elbow_z[6],
         temp_elbow_x[6], temp_elbow_y[6], temp_elbow_z[6],
         temp_target_x[6], temp_target_y[6], temp_target_z[6],
         diff_x2[6], diff_y2[6], diff_z2[6],
         diff_x3[6], diff_y3[6], diff_z3[6],
         temp_target_angle[6], temp_target_angle2[6],
         temp_diffx3[6], temp_diffy3[6], temp_diffz3[6],
         temp_target_angle3[6], temp_target_angle4[6];

  // Start pose for each set of two joints
  for (int i=0; i<6; i++){
    start_x[i] = cos(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436)*(-0.0774);  
    start_y[i] = sin(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436)*(-0.0774);  
    start_z[i] = -0.1057;
  }  

  // Goal pose for each set of two joints
  for (int i=0; i<6; i++)
  {
    temp_x[i] = target_pose.kinematic.position(0) + cos(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.911)*(-0.07825);
    temp_y[i] = target_pose.kinematic.position(1) + sin(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.911)*(-0.07825);
    temp_z[i] = target_pose.kinematic.position(2);
  }

  // Goal Pose for each set of two joints after tool rotation
  goal_orientation = target_pose.kinematic.orientation;
  if (goal_orientation(0,0) && goal_orientation(0,1) && goal_orientation(0,2)
   && goal_orientation(1,0) && goal_orientation(1,1) && goal_orientation(1,2)
   && goal_orientation(2,0) && goal_orientation(2,1) && goal_orientation(2,2))
  {
    goal_orientation(0,0) = 1;
    goal_orientation(1,1) = 1;
    goal_orientation(2,2) = 1;
  }
  for (int i=0; i<6; i++)
  {
    target_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i] + goal_orientation(0,2)*temp_z[i];
    target_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i] + goal_orientation(1,2)*temp_z[i];
    target_z[i] = goal_orientation(2,0)*temp_x[i] + goal_orientation(2,1)*temp_y[i] + goal_orientation(2,2)*temp_z[i];
  }

  // Pose difference for each set of two joints
  for (int i=0; i<6; i++)
  {
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }

  for (int i=0; i<6; i++)
  {
    temp[i] = -diff_x[i]*sin(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436)+diff_y[i]*cos(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436);  
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);

    temp_angle[i] = asin((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    temp_angle2[i] = pow(-1,i%2)*asin(temp[i] / temp2[i]);

    target_angle[i].position = -temp_angle[i] + temp_angle2[i];  
  }
  target_angle[1].position = -target_angle[1].position;  
  target_angle[3].position = -target_angle[3].position; 
  target_angle[5].position = -target_angle[5].position;

  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);
  target_angle_vector.push_back(target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(target_angle[5]);

  // Elbow pose
  for (int i=0; i<6; i++)
  {
    // Adjust motor rotation direction
    target_angle[i].position = -target_angle[i].position;  

    elbow_x[i] = start_x[i] - sin(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436)*link[0]*(-pow(-1,i%2))*cos(target_angle[i].position);
    elbow_y[i] = start_y[i] + cos(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436)*link[0]*(-pow(-1,i%2))*cos(target_angle[i].position);
    elbow_z[i] = start_z[i] + link[0]*(-pow(-1,i%2))*sin(target_angle[i].position);
  
    temp_elbow_x[i] = cos(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*elbow_x[i] - sin(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*elbow_y[i];
    temp_elbow_y[i] = sin(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*elbow_x[i] + cos(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*elbow_y[i];
    temp_elbow_z[i] = elbow_z[i]; 
    temp_target_x[i] = cos(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*target_x[i] - sin(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*target_y[i];
    temp_target_y[i] = sin(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*target_x[i] + cos(-(PI*2.0/3.0*(i/2) - pow(-1,i%2)*0.436))*target_y[i];
    temp_target_z[i] = target_z[i];
  }
  
  for (int i=0; i<6; i++)
  {
    diff_x2[i] = temp_target_x[i] - temp_elbow_x[i];
    diff_y2[i] = temp_target_y[i] - temp_elbow_y[i];
    diff_z2[i] = temp_target_z[i] - temp_elbow_z[i];

    target_angle[2*i+7].position = asin(diff_x2[i] / (pow(-1,i%2)*link[1]));
    target_angle[2*i+6].position = +target_angle[i].position + pow(-1,i%2)*PI + asin(diff_z2[i]/(-link[1]*cos(target_angle[2*i+7].position)*pow(-1,i%2)));

    // Set initial value to 0
    target_angle[2*i+6].position = target_angle[2*i+6].position - pow(-1,i%2)*2.100;
    target_angle[2*i+7].position = target_angle[2*i+7].position - pow(-1,i%2)*0.064;

    target_angle_vector.push_back(target_angle[2*i+6]);
    target_angle_vector.push_back(target_angle[2*i+7]);
  }

  target_angle[18].position = target_pose.kinematic.position(0);
  target_angle[19].position = target_pose.kinematic.position(1);
  target_angle[20].position = target_pose.kinematic.position(2);
  target_angle_vector.push_back(target_angle[18]);
  target_angle_vector.push_back(target_angle[19]);
  target_angle_vector.push_back(target_angle[20]);

  *goal_joint_value = target_angle_vector;
  manipulator->setAllJointValue(target_angle_vector);

  return true;
}







  // for (int i=0; i<6; i++)
  // {
  //   diff_x3[i] = temp_elbow_x[i] - (-0.0774);
  //   diff_y3[i] = temp_elbow_y[i] - 0;
  //   diff_z3[i] = temp_elbow_z[i] - (-0.1057);
  // }

  // for (int i=0; i<6; i++)
  // {
  //   temp_target_angle[i] = acos(diff_z2[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]) * link[0] / link[1]);
  //   temp_target_angle2[i] = acos(diff_z3[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]));

  //   if (i%2 == 0) 
  //   {
  //     target_angle[2*i+6].position = -(temp_target_angle[i] + temp_target_angle2[i]);
  //   } 
  //   else 
  //   {
  //     target_angle[2*i+6].position = (temp_target_angle[i] + temp_target_angle2[i]);
  //   }
  // }

  // for (int i=0; i<6; i++)
  // {
  //   temp_diffx3[i] = diff_x3[i];
  //   temp_diffy3[i] = cos(target_angle[2*i+6].position)*diff_y3[i] - sin(target_angle[2*i+6].position)*diff_z3[i];
  //   temp_diffz3[i] = sin(target_angle[2*i+6].position)*diff_y3[i] + cos(target_angle[2*i+6].position)*diff_z3[i];
  // }

  // for (int i=0; i<6; i++){
  //   temp_target_angle3[i] = acos(diff_x2[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]) * link[0] / link[1]);
  //   temp_target_angle4[i] = acos(temp_diffx3[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]));
  //   target_angle[2*i+7].position = temp_target_angle3[i] - temp_target_angle4[i];
  // }

  // for (int i=0; i<6; i++)
  // {
  //   // target_angle[2*i+6].position = 0;
  //   // target_angle[2*i+7].position = 0;
  //   // target_angle[2*i+6].position = target_angle[2*i+6].position - 2.103914;
  //   // target_angle[2*i+7].position = target_angle[2*i+7].position*75/125.8 + 0.075;
  //   target_angle_vector.push_back(target_angle[2*i+6]);
  //   target_angle_vector.push_back(target_angle[2*i+7]);
  // }
