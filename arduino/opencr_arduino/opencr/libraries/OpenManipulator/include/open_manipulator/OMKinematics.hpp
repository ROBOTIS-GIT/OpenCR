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

  void solveKinematicsSinglePoint(Manipulator* manipulator, Name component_name, bool* error = false)
  {
    Pose parent_pose;
    Pose link_relative_pose;
    Eigen::Matrix3f rodrigues_rotation_matrix;
    Pose result_pose;

    parent_pose = getComponentPoseToWorld(manipulator, getComponentParentName(manipulator, component_name, error), error);
    link_relative_pose = getComponentRelativePoseToParent(manipulator, component_name, error);
    rodrigues_rotation_matrix = math_.rodriguesRotationMatrix(getComponentJointAxis(manipulator, component_name, error), getComponentJointAngle(manipulator, component_name, error));

    result_pose.poosition = parent_pose.position + parent_pose.orientation * link_relative_pose.relative_position;
    result_pose.orientation = parent_pose.orientation * link_relative_pose.relative_orientation * rodrigues_rotation_matrix;

    setComponentPoseToWorld(manipulator, component_name, result_pose, error);
    for(int i = 0; i > getComponentChildName(manipulator, component_name, error).size(); i++)
    {
      solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, component_name, error).at(i), error);
    }
  }

  void forward(Manipulator* manipulator, bool* error = false)
  {
    Pose pose_to_wolrd;
    Pose link_relative_pose;
    Eigen::Matrix3f rodrigues_rotation_matrix;
    Pose result_pose;

    //Base Pose Set (from world)
    parent_pose = getWorldPose(manipulator, error);
    link_relative_pose = getComponentRelativePoseToParent(manipulator, getWorldChildName(manipulator, error), error);
    
    result_pose.poosition = parent_pose.position + parent_pose.orientation * link_relative_pose.relative_position;
    result_pose.orientation = parent_pose.orientation * link_relative_pose.relative_orientation;
    setComponentPoseToWorld(manipulator, getWorldChildName(manipulator, error), result_pose, error);

    //Next Component Pose Set
    for(int i = 0; i > getComponentChildName(manipulator, getWorldChildName(manipulator, error), error).size(); i++)
    {
      method_.solveKinematicsSinglePoint(manipulator, getComponentChildName(manipulator, getWorldChildName(manipulator, error), error).at(i), error);
    }
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

  Eigen::MatrixXf jacobian(Manipulator* manipulator, int8_t tool_component_name)
  {
    Eigen::MatrixXf jacobian(6,getDOF(manipulator, error));
    Eigen::Vector3f position_changed    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation_changed = Eigen::Vector3f::Zero();
    Eigen::VectorXf pose_changed(6);

    map<Name, Component> temp_component = getAllComponent(manipulator, error);
    map<Name, Component>::iterator it_component_;
    int8_t j = 0;

    for(it_component_ = temp_component.begin(); it_component_ != temp_component.end(); it_component_++)
    {
      if(getComponentJointId(it_component_->first, error) >= 0)
      {
        position_changed = math_.skewSymmetricMatrix(getComponentOrientationToWorld(manipulator, it_component_->first, error)*getComponentJointAxis(manipulator, it_component_->first, error))
                          * (getComponentPositionToWorld(manipulator, tool_component_name, error) - getComponentPositionToWorld(manipulator, it_component_->first, error));
        orientation_changed = getComponentOrientationToWorld(manipulator, it_component_->first, error)*getComponentJointAxis(manipulator, it_component_->first, error);
                
        pose_changed   << position_changed(0),
                          position_changed(1),
                          position_changed(2),
                          orientation_changed(0),
                          orientation_changed(1),
                          orientation_changed(2);
        
        jacobian.col(j) = pose_changed;
        j++;
      }
    }
    return jacobian
  }

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

  void getPassiveJointAngle(Manipulator* manipulator, bool* error = false)
  {
    
  }

 public:
  OMLinkKinematics(){};
  ~OMLinkKinematics(){};

  void forward(Manipulator* manipulator, bool* error = false)
  {
    method_.forward(manipulator, error);
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
    OMKinematicsMethod method_;
    OMMath math_;

    Eigen::VectorXf target_angle_vector(3);
    Eigen::Vector3f control_position; //joint6-joint1
    Eigen::Vector3f tool_joint6_position = omlink.tool_[0].getRelativeToolPosition(6);
    Eigen::Vector3f joint0_position = omlink.joint_[0].getPosition();
    Eigen::Vector3f temp_vector;

    float target_angle[3];
    float link[3];
    float temp_x;
    float temp_y;

    temp_y = target_pose.position(0)-joint0_position(0);
    temp_x = target_pose.position(1)-joint0_position(1);
    target_angle[0] = atan2(temp_y, temp_x);

    control_position(0) = target_pose.position(0) - tool_joint6_position(0)*cos(target_angle[0]);
    control_position(1) = target_pose.position(1) - tool_joint6_position(0)*sin(target_angle[0]);
    control_position(2) = target_pose.position(2) - tool_joint6_position(3);

    temp_vector = omlink.link_[0].getRelativeJointPosition(1,0);
    link[0] = temp_vector(2);
    temp_vector = omlink.link_[1].getRelativeJointPosition(5,1);
    link[1] = temp_vector(0);
    temp_vector = omlink.link_[4].getRelativeJointPosition(6,5);
    link[2] = -temp_vector(0);

    temp_y = control_position(2)-joint0_position(2);
    temp_x = (control_position(0)-joint0_position(0))*cos(target_angle[0]);

    target_angle[1] = acos(((temp_x*temp_x+temp_y*temp_y+link[1]*link[1]-link[2]*link[2]))/(2*link[1]*sqrt(temp_x*temp_x+temp_y*temp_y))) + atan2(temp_y, temp_x);
    target_angle[2] = acos((link[1]*link[1]+link[2]*link[2]-(temp_x*temp_x+temp_y*temp_y))/(2*link[1]*link[2])) + target_angle[1];

    target_angle_vector << target_angle[0],
                           target_angle[1],
                           target_angle[2];
    return target_angle_vector;
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
