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

/* Authors: Hye-Jong KIM */

#ifndef OMKINEMATICS_HPP_
#define OMKINEMATICS_HPP_

#include "../../include/open_manipulator/OMAPI.hpp"
#include "../../include/open_manipulator/OMMath.hpp"
#include "../../include/open_manipulator/OMDebug.hpp"

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.
#include <Eigen/Dense>
#include <math.h>
#include <vector>



class OMKinematicsMethod
{
 private:
  OMMath math_;

 public: 
  OMKinematicsMethod(){};
  ~OMKinematicsMethod(){};

  void solveKinematicsSinglePoint(Manipulator* manipulator, char* control_point_name, char* mother_control_point_name, char* link_name, bool* error = false)
  {
    Pose mother_pose;
    RelativePose link_relative_pose;
    Eigen::Matrix3f rodrigues_rotation_matrix;
    Pose result_pose;

    mother_pose = getPose(manipulator, mother_control_point_name, error);
    link_relative_pose = getLinkParameter(manipulator, link_name, control_point_name, mother_control_point_name);
    rodrigues_rotation_matrix = math_.rodriguesRotationMatrix(getJointAxis(manipulator, mother_control_point_name, error), getJointAngle(manipulator, mother_control_point_name, error));

    result_pose.poosition = mother_pose.position + mother_pose.orientation * link_relative_pose.relative_position;
    result_pose.orientation = mother_pose.orientation * link_relative_pose.relative_orientation * rodrigues_rotation_matrix;

    setPose(manipulator, control_point_name, result_pose, error);
  }


  void getBaseJointPose(Manipulator* manipulator, int8_t base_joint_number)
  {
    // Pose temp;
    // temp.position = manipulator.base_.getPosition() + manipulator.base_.getOrientation()*manipulator.base_.getRelativeBaseJointPosition(base_joint_number);
    // temp.orientation = manipulator.base_.getOrientation() * manipulator.base_.getRelativeBaseJointOrientation(base_joint_number) * math_.rodriguesRotationMatrix(manipulator.joint_[base_joint_number].getAxis(), manipulator.joint_[base_joint_number].getAngle());
    // manipulator.joint[base_joint_number].setJointPose(temp);
  }

  void getSinglejointPose(Manipulator* manipulator, int8_t joint_number, int8_t mother_joint_number, int8_t link_number)
  {
    // Pose temp;
    // temp.position = manipulator.joint_[mother_joint_number].getPosition() + manipulator.joint_[mother_joint_number].getOrientation()*manipulator.link_[link_number].getRelativeJointPosition(mother_joint_number,joint_number);
    // temp.orientation = manipulator.joint_[mother_joint_number].getOrientation() * manipulator.link_[link_number].getRelativeJointOrientation(mother_joint_number,joint_number) * math_.rodriguesRotationMatrix(manipulator.joint_[joint_number].getAxis(), manipulator.joint[joint_number].getAngle());
    // manipulator.joint_[joint_number].setPose(temp);
  }

  void getToolPose(Manipulator* manipulator, int8_t tool_number, int8_t mother_joint_number)
  {
    // Pose temp;
    // temp.position = manipulator.joint_[mother_joint_number].getPosition() + manipulator.joint_[mother_joint_number].getOrientation()*manipulator.tool_[tool_number].getRelativeToolPosition(mother_joint_number);
    // temp.orientation = manipulator.joint_[mother_joint_number].getOrientation() * manipulator.tool_[tool_number].getRelativeToolOrientation(mother_joint_number);
    // manipulator.tool_[tool_number].setPose(temp);
  }

  void jacobian(Manipulator* manipulator, int8_t tool_number)
  {
  //   Eigen::MatrixXf jacobian(6,manipulator.getDOF());
  //   Eigen::Vector3f position_changed    = Eigen::Vector3f::Zero();
  //   Eigen::Vector3f orientation_changed = Eigen::Vector3f::Zero();
  //   Eigen::VectorXf pose_changed(6);

  //   int8_t j = 0;
  //   for(int8_t i = 0; i < manipulator.getJointSize(); i++)
  //   {
  //     if(manipulator.joint_[i].getId() >= 0)
  //     {
  //       position_changed = math_.skewSymmetricMatrix(manipulator.joint_[i].getOrientation()*manipulator.joint_[i].getAxis()) * ( manipulator.tool_[tool_number].getPosition() - manipulator.joint_[i].getPosition());
  //       orientation_changed = manipulator.joint_[i].getOrientation()*manipulator.joint_[i].getAxis();
        
  //       pose_changed   << position_changed(0),
  //                         position_changed(1),
  //                         position_changed(2),
  //                         orientation_changed(0),
  //                         orientation_changed(1),
  //                         orientation_changed(2);
        
  //       jacobian.col(j) = pose_changed;
  //       j++;
  //     }
  //   }
  }
};

class OMChainKinematics
{
  private:
    OMMath math_;
    OMKinematicsMethod method_;

  public:
    OMChainKinematics(){};
    ~OMChainKinematics(){};
};

class OMScaraKinematics
{
  private:
    OMMath math_;

  public:
    OMScaraKinematics(){};
    ~OMScaraKinematics(){};

};

class OMLinkKinematics
{
 private:
  OMMath math_;
  OMKinematicsMethod method_;

  // void getPassiveJointAngle(Joint *joint)
  // {
  //   // joint[0].setAngle();
  //   // joint[1].setAngle(); // 0 < joint[1].setAngle < 3PI/4
  //   // joint[2].setAngle(); // PI/2 && joint[1].setAngle + 45? < joint[2].setAngle < PI
  //   joint[3].setAngle(joint[1].getAngle()-joint[2].getAngle());
  //   joint[4].setAngle(-M_PI-(joint[1].getAngle()-joint[2].getAngle()));
  //   joint[5].setAngle(-M_PI-(joint[1].getAngle()-joint[2].getAngle()));
  //   joint[6].setAngle((155 * DEG2RAD) - joint[2].getAngle());
  //   joint[7].setAngle(joint[1].getAngle());
  //   joint[8].setAngle((15 * DEG2RAD)-joint[1].getAngle());
  //   joint[9].setAngle(joint[2].getAngle() - (195 * DEG2RAD));
  //   joint[10].setAngle((90 * DEG2RAD) - joint[2].getAngle());
  // }

 public:
  OMLinkKinematics(){};
  ~OMLinkKinematics(){};

  void forward(Manipulator* manipulator, Eigen::Vector3f aaaa, bool* error = false)
  {

    getPassiveJointAngle(omlink.joint_);

    method_.solveKinematicsSinglePoint(manipulator, aaaa(0), aaaa(1), aaaa(2), error)

 

    // method_.setBasePose(omlink.base_, base_position, base_orientation);
    // method_.getBaseJointPose(omlink,0);
    // method_.getSinglejointPose(omlink, 1, 0, 0);
    // method_.getSinglejointPose(omlink, 2, 0, 0);
    // method_.getSinglejointPose(omlink, 3, 2, 2);
    // method_.getSinglejointPose(omlink, 4, 3, 3);
    // method_.getSinglejointPose(omlink, 5, 1, 1);
    // method_.getSinglejointPose(omlink, 6, 5, 4);
    // method_.getSinglejointPose(omlink, 7, 0, 0);
    // method_.getSinglejointPose(omlink, 8, 7, 5);
    // method_.getSinglejointPose(omlink, 9, 8, 6);
    // method_.getSinglejointPose(omlink, 10, 9, 7);
    // method_.getToolPose(omlink, 0, 6);

  }

  Eigen::VectorXf numericalInverse(Manipulator* manipulator, int8_t tool_number, Pose target_pose, float gain)
  {
    // Eigen::VectorXf target_angle(omlink.getDOF());
    // Eigen::MatrixXf jacobian(6,omlink.getDOF());
    // Eigen::VectorXf differential_pose(6);
    // Eigen::VectorXf previous_differential_pose(6);

    // while(differential_pose.norm() < 1E-6)
    // {
    //   forward(omlink);
    //   jacobian = method_.jacobian(omlink, tool_number);
    //   differential_pose = math_.differentialPose(target_pose.position, omlink.tool[tool_number].getPosition(), target_pose.orientation, omlink.tool[tool_number].getOrientation());
      
    //   Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qrmethod(jacobian);
    //   target_angle = gain * qrmethod.solve(differential_pose);

    //   int8_t k = 0;
    //   for(int8_t j = 0; j < omlink.getJointSize(); j++)
    //   {
    //     if(omlink.joint_[j].getId() >= 0)
    //     {
    //       omlink.joint_[j].setAngle(target_angle(k));
    //       k++;
    //     }
    //   }

    //   if(differential_pose.norm()>previous_differential_pose.norm())
    //   {
    //     //ERROR
    //     break;
    //   }
    //   previous_differential_pose = differential_pose;
    // }
    
    // return target_angle;
  }

  Eigen::VectorXf geometricInverse(Manipulator* manipulator, int8_t tool_number, Pose target_pose, float gain) //for basic model
  {
    // OMKinematicsMethod method_;
    // OMMath math_;

    // Eigen::VectorXf target_angle_vector(3);
    // Eigen::Vector3f control_position; //joint6-joint1
    // Eigen::Vector3f tool_joint6_position = omlink.tool_[0].getRelativeToolPosition(6);
    // Eigen::Vector3f joint0_position = omlink.joint_[0].getPosition();
    // Eigen::Vector3f temp_vector;

    // float target_angle[3];
    // float link[3];
    // float temp_x;
    // float temp_y;

    // temp_y = target_pose.position(0)-joint0_position(0);
    // temp_x = target_pose.position(1)-joint0_position(1);
    // target_angle[0] = atan2(temp_y, temp_x);

    // control_position(0) = target_pose.position(0) - tool_joint6_position(0)*cos(target_angle[0]);
    // control_position(1) = target_pose.position(1) - tool_joint6_position(0)*sin(target_angle[0]);
    // control_position(2) = target_pose.position(2) - tool_joint6_position(3);

    // temp_vector = omlink.link_[0].getRelativeJointPosition(1,0);
    // link[0] = temp_vector(2);
    // temp_vector = omlink.link_[1].getRelativeJointPosition(5,1);
    // link[1] = temp_vector(0);
    // temp_vector = omlink.link_[4].getRelativeJointPosition(6,5);
    // link[2] = -temp_vector(0);

    // temp_y = control_position(2)-joint0_position(2);
    // temp_x = (control_position(0)-joint0_position(0))*cos(target_angle[0]);

    // target_angle[1] = acos(((temp_x*temp_x+temp_y*temp_y+link[1]*link[1]-link[2]*link[2]))/(2*link[1]*sqrt(temp_x*temp_x+temp_y*temp_y))) + atan2(temp_y, temp_x);
    // target_angle[2] = acos((link[1]*link[1]+link[2]*link[2]-(temp_x*temp_x+temp_y*temp_y))/(2*link[1]*link[2])) + target_angle[1];

    // target_angle_vector << target_angle[0],
    //                        target_angle[1],
    //                        target_angle[2];
    // return target_angle_vector;
  }
};

class OMDeltaKinematics
{
  private:
    OMMath math_;

  public:
    OMDeltaKinematics(){};
    ~OMDeltaKinematics(){};

};

class MYKinematics
{
 private:
   OMMath math_;

 public:
   MYKinematics(){};
   ~MYKinematics(){};
};

#endif // OMKINEMATICS_HPP_
