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

#include "../include/open_manipulator_libs/Delta_Kinematics.h"

using namespace ROBOTIS_MANIPULATOR; 
using namespace DELTA_KINEMATICS;

void Delta::setOption(const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  if(get_arg_[0] =="inverse_solver")
  {
    STRING inverse_kinematics_solver_option = get_arg_[1];
    inverse_kinematics_solver_option_ = inverse_kinematics_solver_option;
  }
}

void Delta::updatePassiveJointValue(Manipulator *manipulator){}

Eigen::MatrixXd Delta::jacobian(Manipulator *manipulator, Name tool_name) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  return jacobian;
}

void Delta::forwardKinematics(Manipulator *manipulator)  
{
  forwardKinematicsSolverUsingGeometry(manipulator, manipulator->getWorldChildName());
}

bool Delta::inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double> *goal_joint_value)
{
  return inverseKinematicsSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}


/*-----------------------------------------------------------------------------------------------*/

void Delta::forwardKinematicsSolverUsingGeometry(Manipulator *manipulator, Name component_name)  
{
  // Name my_name = component_name;
  // Name parent_name = manipulator->getComponentParentName(my_name);
  // int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  // Eigen::Vector3d parent_position_to_world, my_position_to_world;
  // Eigen::Matrix3d parent_orientation_to_world, my_orientation_to_world;

  // if (parent_name == manipulator->getWorldName())
  // {
  //   parent_position_to_world = manipulator->getWorldPosition();
  //   parent_orientation_to_world = manipulator->getWorldOrientation();
  // }
  // else
  // {
  //   parent_position_to_world = manipulator->getComponentPositionToWorld(parent_name);
  //   parent_orientation_to_world = manipulator->getComponentOrientationToWorld(parent_name);
  // }

  // my_position_to_world = parent_orientation_to_world * manipulator->getComponentRelativePositionToParent(my_name) + parent_position_to_world;
  // my_orientation_to_world = parent_orientation_to_world * RM_MATH::rodriguesRotationMatrix(manipulator->getJointAxis(my_name), manipulator->getJointValue(my_name));

  // manipulator->setComponentPositionToWorld(my_name, my_position_to_world);
  // manipulator->setComponentOrientationToWorld(my_name, my_orientation_to_world);

  // for (int8_t index = 0; index < number_of_child; index++)
  // {
  //   Name child_name = manipulator->getComponentChildName(my_name).at(index);
  //   forward(manipulator, child_name);
  // }

  //---------- Or ----------//

  //   Pose pose_to_wolrd;
  // Pose link_relative_pose;
  // Matrix3d rodrigues_rotation_matrix;
  // Pose result_pose;

  // // getComponentJointAngle(manipulator, getWorldChildName(manipulator)
  // double theta[3] = {0,0,0}; 

  // // Link Lengths
  // double link[2];
  // link[0] = 0.100f;
  // link[1] = 0.217f;

  // double start_x[3];       
  // double start_y[3];
  // double start_z[3];
  // // Start pose for each set of two joints
  // for (int i=0; i<3; i++){
  //   start_x[i] = cos(PI*2.0/3.0*i)*(-0.055f);
  //   start_y[i] = sin(PI*2.0/3.0*i)*(-0.055f);
  //   // start_z[i] = 0.169894;
  //   start_z[i] = 0;
  // }

  // double x_elb[3];       
  // double y_elb[3];
  // double z_elb[3];  // not elbow but moved elbows 
  // for (int i=0; i<3; i++){
  //   x_elb[i] = start_x[i] + cos(PI*2.0/3.0*i) * (-link[0]*cos(theta[i]) + 0.020f);
  //   y_elb[i] = start_y[i] + sin(PI*2.0/3.0*i) * (-link[0]*cos(theta[i]) + 0.020f);
  //   z_elb[i] = start_z[i] + (-link[0] * sin(theta[i]));
  // }
  
  // double x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3;

  // x1 = x_elb[0];
  // x2 = x_elb[1];
  // x3 = x_elb[2];
  // y1 = y_elb[0];
  // y2 = y_elb[1];
  // y3 = y_elb[2];
  // z1 = z_elb[0];
  // z2 = z_elb[1];
  // z3 = z_elb[2];

  // double dnm = (x1-x3)*(y1-y2)+(x1-x2)*(y1-y3);
 
  // double w1 = y1*y1 + z1*z1;
  // double w2 = x2*x2 + y2*y2 + z2*z2;
  // double w3 = x3*x3 + y3*y3 + z3*z3;

  // // x = (a1*z + b1)/dnm
  // double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  // double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
  // // y = (a2*z + b2)/dnm;
  // double a2 = -(z2-z1)*x3+(z3-z1)*x2;
  // double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
  
  // // a*z^2 + b*z + c = 0
  // double a = a1*a1 + a2*a2 + dnm*dnm;
  // double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  // double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - link[1]*link[1]);
  
  // // discriminant
  // double d = b*b - (double)4.0*a*c;
  // // if (d < 0) return -1; // non-existing point
  // if (d < 0) Serial.println("not existing"); // non-existing point
 
  // z0 = -(double)0.5*(b+sqrt(d))/a;
  // x0 = (a1*z0 + b1)/dnm;
  // y0 = (a2*z0 + b2)/dnm;

  // result_pose.position(0) = x0; 
  // result_pose.position(1) = y0; 
  // result_pose.position(2) = z0 + 0.169894; 

}

bool Delta::inverseKinematicsSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<double> *goal_joint_value)
{
  std::vector<double> target_angle_vector;

  double temp_angle[3];
  double temp_angle2[3];
  double target_angle[3];
  double target_angle2[3];
  double target_angle3[3];
  double link[2];
  double start_x[3];       
  double start_y[3];
  double start_z[3];
  double target_x[3];       
  double target_y[3];
  double target_z[3];
  double diff_x[3];       
  double diff_y[3];
  double diff_z[3];
  double temp[3];
  double temp2[3];
  double target_pose_length[3];

  // Link Lengths
  link[0] = 0.100f;
  link[1] = 0.225f;
  link[2] = 0.020f;

  // Start pose for each set of two joints
  for (int i=0; i<3; i++){
    start_x[i] = cos(PI*2.0/3.0*i)*(-0.055f);
    start_y[i] = sin(PI*2.0/3.0*i)*(-0.055f);
    start_z[i] = 0.180;
  }
  
  // Goal pose for each set of two joints
  for (int i=0; i<3; i++){
    target_x[i] = target_pose.position(0) + cos(PI*2.0/3.0*i)*(-link[2]);
    target_y[i] = target_pose.position(1) + sin(PI*2.0/3.0*i)*(-link[2]);
    target_z[i] = target_pose.position(2);
  }

  // Pose difference for each set of two joints
  for (int i=0; i<3; i++){
    diff_x[i] = target_x[i] - start_x[i];
    diff_y[i] = target_y[i] - start_y[i];
    diff_z[i] = target_z[i] - start_z[i];
  }

  for (int i=0; i<3; i++){
    temp[i] = diff_x[i]*cos(PI*2.0/3.0*i)+diff_y[i]*sin(PI*2.0/3.0*i);
    temp2[i] = sqrt(temp[i]*temp[i] + diff_z[i]*diff_z[i]);
    temp_angle[i] = acos((link[1]*link[1] - link[0]*link[0] - diff_x[i]*diff_x[i] - diff_y[i]*diff_y[i] - diff_z[i]*diff_z[i])
                    / (2.0*link[0]*temp2[i]));
    temp_angle2[i] = acos(temp[i] / temp2[i]);
    target_angle[i] = +temp_angle[i] - temp_angle2[i];  
  }

  double elbow_x[3];
  double elbow_y[3];
  double elbow_z[3];

  for (int i=0; i<3; i++){
    elbow_x[i] = start_x[i] - cos(PI*2.0/3.0*i)*link[0]*cos(target_angle[i]);
    elbow_y[i] = start_y[i] - sin(PI*2.0/3.0*i)*link[0]*cos(target_angle[i]);
    elbow_z[i] = start_z[i] - link[0]*sin(target_angle[i]);
  }

  double diff_x2[3];       
  double diff_y2[3];
  double diff_z2[3];

  // Pose difference for each set of two joints
  for (int i=0; i<3; i++){
    diff_x2[i] = target_x[i] - elbow_x[i];
    diff_y2[i] = target_y[i] - elbow_y[i];
    diff_z2[i] = target_z[i] - elbow_z[i];
  }

  double temp3[3];
  double temp4[3];
  for (int i=0; i<3; i++){
    // temp3[i] = sqrt(diff_x2[i]*diff_x2[i]+diff_y[i]*diff_y[i]+diff_z[i]*diff_z[i]);
    target_angle[2*i+3] = PI/2 + acos(-diff_z2[i]/link[1]) - target_angle[i];
    temp3[i] = diff_x2[i]*cos(PI*2.0/3.0*i)+diff_y2[i]*sin(PI*2.0/3.0*i); 
    temp4[i] =-diff_x2[i]*sin(PI*2.0/3.0*i)+diff_y2[i]*cos(PI*2.0/3.0*i); 
    target_angle[2*i+4] = asin(temp4[i] / sqrt(link[1]*link[1]-diff_z2[i]*diff_z2[i]));
    // target_angle[2*i+4] = acos((temp4[i])/ (link[1]*acos(-diff_z2[i])/link[1])));
    // target_angle[i+3] = PI - acos((link[0]^link[0]+link[1]^link[1]-temp_link[i]^temp_link[i]]) / (2*link[0]*link[1]));
  }

  target_angle2[0] = target_angle[4];
  target_angle2[1] = target_angle[6];
  target_angle2[2] = target_angle[8];

  // ctrl_joint_angle[3] = target_angle[3];
  // ctrl_joint_angle[4] = target_angle[4];
  // ctrl_joint_angle[5] = target_angle[5];
  // ctrl_joint_angle[6] = target_angle[3];
  // ctrl_joint_angle[7] = target_angle[4];
  // ctrl_joint_angle[8] = target_angle[5];
  // ctrl_joint_angle[9] = PI - target_angle[3]; 
  // ctrl_joint_angle[10] = -target_angle[4]; 

  // Set Joint Angle 
  // Serial.println(temp[0],4);
  // Serial.println(temp2[0],4);
  // Serial.println(diff_z[0],4);
  // Serial.println(link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0], 4);
  // Serial.println((link[1]*link[1] - link[0]*link[0] - diff_x[0]*diff_x[0] - diff_y[0]*diff_y[0] - diff_z[0]*diff_z[0])/(-2.0*link[0]*temp2[0]),4);
  // Serial.println(temp_angle[0],4);
  // Serial.println(temp_angle2[0],4);
  Serial.println(target_angle[0],4);
  Serial.println(target_angle[1],4);
  Serial.println(target_angle[2],4);
  Serial.println(temp4[0],4);
  Serial.println((link[1]*link[1]-diff_z2[0]*diff_z2[0]),4);
  Serial.println(temp4[0] / (link[1]*link[1]-diff_z2[0]*diff_z2[0]),4);


  Serial.println(target_angle2[0],4);
  Serial.println(target_angle2[1],4);
  Serial.println(target_angle2[2],4);
  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);

  // double felbow_x[3];       
  // double felbow_y[3];
  // double felbow_z[3];

  // double error = 1000000;
  // double ferroreach[3];
  // double temp_error;
  // double result[3];
  // for (int j=-70; j<70; j++){
  //   for (int k=-70; k<70; k++){
  //     for (int l=-30; l<30; l++){
  //       for (int i=0; i<3; i++){
  //         felbow_x[i] = cos(PI*2.0/3.0*i)*(0.033f) + link[0]*cos(target_angle2[i])*cos(PI*2.0/3.0*i);
  //         felbow_y[i] = sin(PI*2.0/3.0*i)*(0.033f) + link[0]*cos(target_angle2[i])*sin(PI*2.0/3.0*i);
  //         felbow_z[i] = -0.169894 + link[0]*sin(target_angle2[i]);
          
  //         ferroreach[i] = abs((felbow_x[i]+j/1000.0f)*(felbow_x[i]+j/1000.0f)
  //                      + (felbow_y[i]+k/1000.0f)*(felbow_y[i]+k/1000.0f) 
  //                      + (felbow_z[i]+l/1000.0f)*(felbow_z[i]+l/1000.0f) 
  //                      - link[1]*link[1]);
                       
  //       }
  //       temp_error = ferroreach[0] + ferroreach[1] + ferroreach[2];
  //       // Serial.println(temp_error);
        
  //       if (temp_error < error){
  //         error = temp_error;
  //         // Serial.println(error);
  //         // Serial.println(j);
  //         // Serial.println(k);
  //         // Serial.println(l);
  //         result[0] = j;
  //         result[1] = k;
  //         result[2] = l;
  //       }
  //     }
  //   }
  // }

  // Serial.println(result[0]);
  // Serial.println(result[1]);
  // Serial.println(result[2]);
  goal_joint_value = &target_angle_vector;
  return true;
}