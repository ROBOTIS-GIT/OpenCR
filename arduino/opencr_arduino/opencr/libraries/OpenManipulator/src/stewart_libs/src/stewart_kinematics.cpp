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
  std::vector<JointValue> target_joint_value;

  double temp_angle[6];
  double temp_angle2[6];
  JointValue target_angle[20];
  double link[2];
  double start_x[6], start_y[6], start_z[6],
         temp_x[6], temp_y[6], temp_z[6],
         target_x[6], target_y[6], target_z[6],
         diff_x[6], diff_y[6], diff_z[6];
  double temp[6];
  double temp2[6];
  double target_pose_length[6];
  Matrix3d goal_orientation;

  // Link Lengths
  link[0] = 0.026;    // modified the values
  link[1] = 0.1227;   // modified the values

  // Start pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      start_x[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*(-0.0774);  // angle :25degrees
      start_y[i] = sin(PI*2.0/3.0*(i/2) - 0.436)*(-0.0774);  // modified the values
    } 
    else {
      start_x[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*(-0.0774);  // modified the values
      start_y[i] = sin(PI*2.0/3.0*(i/2) + 0.436)*(-0.0774);  // modified the values
    }
    start_z[i] = -0.1057; // modified the values
  }  

  // Goal pose for each set of two joints
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      temp_x[i] = target_pose.kinematic.position(0) + cos(PI*2.0/3.0*(i/2) - 0.911)*(-0.07825);
      temp_y[i] = target_pose.kinematic.position(1) + sin(PI*2.0/3.0*(i/2) - 0.911)*(-0.07825);
    } 
    else {
      temp_x[i] = target_pose.kinematic.position(0) + cos(PI*2.0/3.0*(i/2) + 0.911)*(-0.07825);
      temp_y[i] = target_pose.kinematic.position(1) + sin(PI*2.0/3.0*(i/2) + 0.911)*(-0.07825);
    }
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
  for (int i=0; i<6; i++){
    target_x[i] = goal_orientation(0,0)*temp_x[i] + goal_orientation(0,1)*temp_y[i] + goal_orientation(0,2)*temp_z[i];
    target_y[i] = goal_orientation(1,0)*temp_x[i] + goal_orientation(1,1)*temp_y[i] + goal_orientation(1,2)*temp_z[i];
    target_z[i] = goal_orientation(2,0)*temp_x[i] + goal_orientation(2,1)*temp_y[i] + goal_orientation(2,2)*temp_z[i];
  }

  // Pose difference for each set of two joints
  for (int i=0; i<6; i++){
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }

  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      temp[i] = -diff_x[i]*sin(PI*2.0/3.0*(i/2) - 0.436)+diff_y[i]*cos(PI*2.0/3.0*(i/2) - 0.436);  
    } 
    else {
      temp[i] = -diff_x[i]*sin(PI*2.0/3.0*(i/2) + 0.436)+diff_y[i]*cos(PI*2.0/3.0*(i/2) + 0.436);  
    }
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);

    temp_angle[i] = asin((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    if (i%2 == 0) {
      temp_angle2[i] = asin(temp[i] / temp2[i]);
    } 
    else {
      temp_angle2[i] = -asin(temp[i] / temp2[i]);
    }

    target_angle[i].position = -temp_angle[i] + temp_angle2[i];  
  }
  target_angle[1].position = -target_angle[1].position;  
  target_angle[3].position = -target_angle[3].position; 
  target_angle[5].position = -target_angle[5].position;

  target_joint_value.push_back(target_angle[0]);
  target_joint_value.push_back(target_angle[1]);
  target_joint_value.push_back(target_angle[2]);
  target_joint_value.push_back(target_angle[3]);
  target_joint_value.push_back(target_angle[4]);
  target_joint_value.push_back(target_angle[5]);

  *goal_joint_value = target_joint_value;


  double elbow_x[6], elbow_y[6], elbow_z[6];
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      elbow_x[i] = start_x[i] - sin(PI*2.0/3.0*(i/2) - 0.436)*link[0]*(-cos(target_angle[i].position)); 
      elbow_y[i] = start_y[i] + cos(PI*2.0/3.0*(i/2) - 0.436)*link[0]*(-cos(target_angle[i].position)); 
      elbow_z[i] = start_z[i] + link[0]*sin(target_angle[i].position); 
    } 
    else {
      elbow_x[i] = start_x[i] - sin(PI*2.0/3.0*(i/2) + 0.436)*link[0]*(+cos(target_angle[i].position));  
      elbow_y[i] = start_y[i] + cos(PI*2.0/3.0*(i/2) + 0.436)*link[0]*(+cos(target_angle[i].position));  
      elbow_z[i] = start_z[i] + link[0]*sin(target_angle[i].position); 
    }
  }  



  double elbow_x2[6], elbow_y2[6], elbow_z2[6],
         target_x2[6], target_y2[6], target_z2[6],
         diff_x2[6], diff_y2[6], diff_z2[6],
         diff_x3[6], diff_y3[6], diff_z3[6];

  // Elbow pose  
  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      elbow_x2[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*elbow_x[i] + sin(PI*2.0/3.0*(i/2) - 0.436)*elbow_y[i];  
      elbow_y2[i] =-sin(PI*2.0/3.0*(i/2) - 0.436)*elbow_x[i] + cos(PI*2.0/3.0*(i/2) - 0.436)*elbow_y[i];  
    } 
    else {
      elbow_x2[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*elbow_x[i] + sin(PI*2.0/3.0*(i/2) + 0.436)*elbow_y[i];  
      elbow_y2[i] =-sin(PI*2.0/3.0*(i/2) + 0.436)*elbow_x[i] + cos(PI*2.0/3.0*(i/2) + 0.436)*elbow_y[i];  
    }
    elbow_z2[i] = elbow_z[i]; 
  }  

  for (int i=0; i<6; i++){
    diff_x3[i] = elbow_x2[i] - (-0.0774);
    diff_y3[i] = elbow_y2[i] - 0;
    diff_z3[i] = elbow_z2[i] - (-0.1057);
  }

  for (int i=0; i<6; i++){
    if (i%2 == 0) {
      target_x2[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*target_x[i] + sin(PI*2.0/3.0*(i/2) - 0.436)*target_y[i];
      target_y2[i] =-sin(PI*2.0/3.0*(i/2) - 0.436)*target_x[i] + cos(PI*2.0/3.0*(i/2) - 0.436)*target_y[i];
    } 
    else {
      target_x2[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*target_x[i] + sin(PI*2.0/3.0*(i/2) + 0.436)*target_y[i];
      target_y2[i] =-sin(PI*2.0/3.0*(i/2) + 0.436)*target_x[i] + cos(PI*2.0/3.0*(i/2) + 0.436)*target_y[i];
    }
    target_z2[i] = target_z[i];
  }

  for (int i=0; i<6; i++){
    diff_x2[i] = target_x2[i] - elbow_x2[i];
    diff_y2[i] = target_y2[i] - elbow_y2[i];
    diff_z2[i] = target_z2[i] - elbow_z2[i];
  }

  // Serial.println("Diffs");
  // Serial.print(elbow_x2[4],4);
  // Serial.print(elbow_x2[0],4);
  // Serial.print(elbow_x2[1],4);
  // Serial.print(elbow_x2[2],4);
  // Serial.print(elbow_x2[3],4);
  // Serial.println(elbow_x2[5],4);
  // Serial.print(elbow_y2[0],4);
  // Serial.print(elbow_y2[1],4);
  // Serial.print(elbow_y2[2],4);
  // Serial.print(elbow_y2[3],4);
  // Serial.print(elbow_y2[4],4);
  // Serial.println(elbow_y2[5],4);
  // Serial.print(elbow_z2[0],4);
  // Serial.print(elbow_z2[1],4);
  // Serial.print(elbow_z2[2],4);
  // Serial.print(elbow_z2[3],4);
  // Serial.print(elbow_z2[4],4);
  // Serial.println(elbow_z2[5],4);

  // Serial.println("Diffs");
  // Serial.print(diff_x2[0],4);
  // Serial.print(diff_x2[1],4);
  // Serial.print(diff_x2[2],4);
  // Serial.print(diff_x2[3],4);
  // Serial.print(diff_x2[4],4);
  // Serial.println(diff_x2[5],4);
  // Serial.print(diff_y2[0],4);
  // Serial.print(diff_y2[1],4);
  // Serial.print(diff_y2[2],4);
  // Serial.print(diff_y2[3],4);
  // Serial.print(diff_y2[4],4);
  // Serial.println(diff_y2[5],4);
  // Serial.print(diff_z2[0],4);
  // Serial.print(diff_z2[1],4);
  // Serial.print(diff_z2[2],4);
  // Serial.print(diff_z2[3],4);
  // Serial.print(diff_z2[4],4);
  // Serial.println(diff_z2[5],4);

  // Serial.println("Diffs");
  // Serial.print(diff_x3[0],4);
  // Serial.print(diff_x3[1],4);
  // Serial.print(diff_x3[2],4);
  // Serial.print(diff_x3[3],4);
  // Serial.print(diff_x3[4],4);
  // Serial.println(diff_x3[5],4);
  // Serial.print(diff_y3[0],4);
  // Serial.print(diff_y3[1],4);
  // Serial.print(diff_y3[2],4);
  // Serial.print(diff_y3[3],4);
  // Serial.print(diff_y3[4],4);
  // Serial.println(diff_y3[5],4);
  // Serial.print(diff_z3[0],4);
  // Serial.print(diff_z3[1],4);
  // Serial.print(diff_z3[2],4);
  // Serial.print(diff_z3[3],4);
  // Serial.print(diff_z3[4],4);
  // Serial.println(diff_z3[5],4);


  // double temp_target_angle[6];
  // double temp_target_angle2[6];

  // for (int i=0; i<6; i++){
  //   temp_target_angle[i] = acos(diff_z2[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]) * link[0] / link[1]);
  //   temp_target_angle2[i] = acos(diff_z3[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]));

  //   if (i%2 == 0) {
  //     target_angle[2*i+6] = -(temp_target_angle[i] + temp_target_angle2[i]);
  //   } 
  //   else {
  //     target_angle[2*i+6] = (temp_target_angle[i] + temp_target_angle2[i]);
  //   }
  // }

  // // Serial.println("Diffs");
  // // Serial.print(temp_target_angle[0],6);
  // // Serial.print(temp_target_angle[1],6);
  // // Serial.print(temp_target_angle[2],6);
  // // Serial.print(temp_target_angle[3],6);
  // // Serial.print(temp_target_angle[4],6);
  // // Serial.println(temp_target_angle[5],6);
  // // Serial.print(temp_target_angle2[0],6);
  // // Serial.print(temp_target_angle2[1],6);
  // // Serial.print(temp_target_angle2[2],6);
  // // Serial.print(temp_target_angle2[3],6);
  // // Serial.print(temp_target_angle2[4],6);
  // // Serial.println(temp_target_angle2[5],6);

  // double temp_diffx3[6];
  // double temp_diffy3[6];
  // double temp_diffz3[6];

  // // Serial.println("Diffstemp_diffy3");
  // for (int i=0; i<6; i++){
  //   temp_diffx3[i] = diff_x3[i];
  //   temp_diffy3[i] = cos(target_angle[2*i+6])*diff_y3[i] - sin(target_angle[2*i+6])*diff_z3[i];
  //   temp_diffz3[i] = sin(target_angle[2*i+6])*diff_y3[i] + cos(target_angle[2*i+6])*diff_z3[i];
  //   // Serial.print(temp_diffz3[i] - diff_z2[i] * link[0] / link[1],6);
  //   // Serial.print(" ");
  // }
  // // Serial.println(" ");

  // double temp_target_angle3[6];
  // double temp_target_angle4[6];

  // // // Serial.println("Diffs");
  // // for (int i=0; i<6; i++){
  // //   temp_target_angle3[i] = asin(diff_y2[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]) * link[0] / link[1]);
  // //   temp_target_angle4[i] = asin(temp_diffy3[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]));
  // //   target_angle[2*i+7] = temp_target_angle3[i] - temp_target_angle4[i];
  // //   // Serial.print(target_angle[2*i+7],6);
  // //   // Serial.print(sin(target_angle[2*i+7])*temp_diffx3[i]+cos(target_angle[2*i+7])*temp_diffy3[i] - diff_y2[i] * link[0] / link[1],6);
  // //   // Serial.print(" ");
  // // }
  // // // Serial.println(" ");

  // // Serial.println("Diffs");
  // for (int i=0; i<6; i++){
  //   temp_target_angle3[i] = acos(diff_x2[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]) * link[0] / link[1]);
  //   temp_target_angle4[i] = acos(temp_diffx3[i] / sqrt(temp_diffx3[i]*temp_diffx3[i] + temp_diffy3[i]*temp_diffy3[i]));
  //   target_angle[2*i+7] = temp_target_angle3[i] - temp_target_angle4[i];
  //   // Serial.print(temp_target_angle3[i],6);
  //   // Serial.print(target_angle[2*i+7],6);
  //   // Serial.print(cos(target_angle[2*i+7]/2)*diff_x3[i] - diff_x2[i] * link[0] / link[1],6);
  //   // Serial.print(" ");
  // }
  // // Serial.println(" ");


  // Serial.println("Diffs");
  // Serial.print(temp_target_angle3[0],4);
  // Serial.print(temp_target_angle3[1],4);
  // Serial.print(temp_target_angle3[2],4);
  // Serial.print(temp_target_angle3[3],4);
  // Serial.print(temp_target_angle3[4],4);
  // Serial.println(temp_target_angle3[5],4);
  // Serial.print(temp_target_angle4[0],4);
  // Serial.print(temp_target_angle4[1],4);
  // Serial.print(temp_target_angle4[2],4);
  // Serial.print(temp_target_angle4[3],4);
  // Serial.print(temp_target_angle4[4],4);
  // Serial.println(temp_target_angle4[5],4);


  // Serial.println("Diffs");
  // Serial.print(target_angle[6],6);
  // Serial.print(target_angle[7],4);
  // Serial.print(target_angle[8],6);
  // Serial.print(target_angle[9],4);
  // Serial.print(target_angle[10],6);
  // Serial.print(target_angle[11],4);
  // Serial.print(target_angle[12],4);
  // Serial.print(target_angle[13],4);
  // Serial.print(target_angle[14],4);
  // Serial.print(target_angle[15],4);
  // Serial.print(target_angle[16],4);
  // Serial.print(target_angle[17],4);
  // Serial.println("");
  return true;
}







  // double elbow_x2[6], elbow_y2[6], elbow_z2[6],
  //        target_x2[6], target_y2[6], target_z2[6],
  //        diff_x2[6], diff_y2[6], diff_z2[6],
  //        diff_x3[6], diff_y3[6], diff_z3[6];

  // // Elbow pose  
  // for (int i=0; i<6; i++){
  //   if (i%2 == 0) {
  //     elbow_x2[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*elbow_x[i] + sin(PI*2.0/3.0*(i/2) - 0.436)*elbow_y[i];  
  //     elbow_y2[i] =-sin(PI*2.0/3.0*(i/2) - 0.436)*elbow_x[i] + cos(PI*2.0/3.0*(i/2) - 0.436)*elbow_y[i];  
  //   } 
  //   else {
  //     elbow_x2[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*elbow_x[i] + sin(PI*2.0/3.0*(i/2) + 0.436)*elbow_y[i];  
  //     elbow_y2[i] =-sin(PI*2.0/3.0*(i/2) + 0.436)*elbow_x[i] + cos(PI*2.0/3.0*(i/2) + 0.436)*elbow_y[i];  
  //   }
  //   elbow_z2[i] = elbow_z[i]; 
  // }  

  // for (int i=0; i<6; i++){
  //   diff_x3[i] = elbow_x2[i] - (-0.774);
  //   diff_y3[i] = elbow_y2[i] - 0;
  //   diff_z3[i] = elbow_z2[i] - (-0.1051);
  // }

  // for (int i=0; i<6; i++){
  //   if (i%2 == 0) {
  //     target_x2[i] = cos(PI*2.0/3.0*(i/2) - 0.436)*target_x[i] + sin(PI*2.0/3.0*(i/2) - 0.436)*target_y[i];
  //     target_y2[i] =-sin(PI*2.0/3.0*(i/2) - 0.436)*target_x[i] + cos(PI*2.0/3.0*(i/2) - 0.436)*target_y[i];
  //   } 
  //   else {
  //     target_x2[i] = cos(PI*2.0/3.0*(i/2) + 0.436)*target_x[i] + sin(PI*2.0/3.0*(i/2) + 0.436)*target_y[i];
  //     target_y2[i] =-sin(PI*2.0/3.0*(i/2) + 0.436)*target_x[i] + cos(PI*2.0/3.0*(i/2) + 0.436)*target_y[i];
  //   }
  //   target_z2[i] = target_z[i];
  // }

  // for (int i=0; i<6; i++){
  //   diff_x2[i] = target_x2[i] - elbow_x2[i];
  //   diff_y2[i] = target_y2[i] - elbow_y2[i];
  //   diff_z2[i] = target_z2[i] - elbow_z2[i];

  //   target_angle[2*i+7] = atan2(abs(diff_x2[i]),abs(diff_y2[i]));
  //   target_angle[2*i+6] = asin(abs(diff_y2[i])/link[1]/cos(target_angle[2*i+7]));    
  // }




  // Eigen::Matrix3d temp_matrix;
  // Eigen::Matrix4d temp_matrix2;

  // for (int i=0; i<6; i++){  
  //   temp_matrix << cos(target_angle[2*i+7]), -sin(target_angle[2*i+7]), 0,
  //                  sin(target_angle[2*i+7]),  cos(target_angle[2*i+7]), 0,
  //                  0                       ,  0                       , 1;

  //   temp_matrix2 = RM_MATH::getInverseTransformation(temp_matrix);
  // }

  // double temp_diffz2[6];

  // // Serial.println("Diffs");
  // for (int i=0; i<6; i++){
  //   temp_diffz2[i] = temp_matrix2(2,0)*diff_x2[i] + temp_matrix2(2,1)*diff_y2[i] + temp_matrix2(2,2)*diff_z2[i];
  //   // Serial.print(temp_diffz2[i] - diff_z2[i],4);
  // }
  // // Serial.println("");

  // double temp_target_angle3[6];
  // double temp_target_angle4[6];

  // for (int i=0; i<6; i++){
  //   temp_target_angle3[i] = asin(temp_diffz2[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]) * link[0] / link[1]);
  //   temp_target_angle4[i] = asin(diff_z3[i] / sqrt(diff_y3[i]*diff_y3[i] + diff_z3[i]*diff_z3[i]));
  //   target_angle[2*i+7] = temp_target_angle3[i] - temp_target_angle4[i];
  // }



