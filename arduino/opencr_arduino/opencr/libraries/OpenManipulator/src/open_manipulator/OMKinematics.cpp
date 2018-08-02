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

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../../include/open_manipulator/OMKinematics.h"


void OMKinematicsMethod::solveKinematicsSinglePoint(Manipulator* manipulator, Name component_name, bool* error = false)
{
  Pose parent_pose;
  Pose link_relative_pose;
  Eigen::Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  parent_pose = MANAGER::getComponentPoseToWorld(manipulator, MANAGER::getComponentParentName(manipulator, component_name, error), error);
  link_relative_pose = MANAGER::getComponentRelativePoseToParent(manipulator, component_name, error);
  rodrigues_rotation_matrix = MATH::rodriguesRotationMatrix(getComponentJointAxis(manipulator, component_name, error), MANAGER::getComponentJointAngle(manipulator, component_name, error));

  result_pose.poosition = parent_pose.position + parent_pose.orientation * link_relative_pose.relative_position;
  result_pose.orientation = parent_pose.orientation * link_relative_pose.relative_orientation * rodrigues_rotation_matrix;

  MANAGER::setComponentPoseToWorld(manipulator, component_name, result_pose, error);
  for(int i = 0; i > MANAGER::getComponentChildName(manipulator, component_name, error).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, MANAGER::getComponentChildName(manipulator, component_name, error).at(i), error);
  }
}

void OMKinematicsMethod::forward(Manipulator* manipulator, bool* error = false)
{
  Pose pose_to_wolrd;
  Pose link_relative_pose;
  Eigen::Matrix3f rodrigues_rotation_matrix;
  Pose result_pose;

  //Base Pose Set (from world)
  parent_pose = MANAGER::getWorldPose(manipulator, error);
  link_relative_pose = MANAGER::getComponentRelativePoseToParent(manipulator, MANAGER::getWorldChildName(manipulator, error), error);
    
  result_pose.poosition = parent_pose.position + parent_pose.orientation * link_relative_pose.relative_position;
  result_pose.orientation = parent_pose.orientation * link_relative_pose.relative_orientation;
  MANAGER::setComponentPoseToWorld(manipulator, MANAGER::getWorldChildName(manipulator, error), result_pose, error);

  //Next Component Pose Set
  for(int i = 0; i > MANAGER::getComponentChildName(manipulator, MANAGER::getWorldChildName(manipulator, error), error).size(); i++)
  {
    solveKinematicsSinglePoint(manipulator, MANAGER::getComponentChildName(manipulator, MANAGER::getWorldChildName(manipulator, error), error).at(i), error);
  }
}

Eigen::MatrixXf OMKinematicsMethod::jacobian(Manipulator* manipulator, int8_t tool_component_name, bool *error = false)
{
  Eigen::MatrixXf jacobian(6,MANAGER::getDOF(manipulator, error));
  Eigen::Vector3f position_changed    = Eigen::Vector3f::Zero();
  Eigen::Vector3f orientation_changed = Eigen::Vector3f::Zero();
  Eigen::VectorXf pose_changed(6);
   map<Name, Component> temp_component = MANAGER::getAllComponent(manipulator, error);
  map<Name, Component>::iterator it_component_;
  int8_t j = 0;

  for(it_component_ = temp_component.begin(); it_component_ != temp_component.end(); it_component_++)
  {
    if(MANAGER::getComponentJointId(it_component_->first, error) >= 0)
    {
      position_changed = MATH::skewSymmetricMatrix(getComponentOrientationToWorld(manipulator, it_component_->first, error)*MANAGER::getComponentJointAxis(manipulator, it_component_->first, error))
                        * (MANAGER::getComponentPositionToWorld(manipulator, tool_component_name, error) - MANAGER::getComponentPositionToWorld(manipulator, it_component_->first, error));
      orientation_changed = MANAGER::getComponentOrientationToWorld(manipulator, it_component_->first, error)*MANAGER::getComponentJointAxis(manipulator, it_component_->first, error);
              
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

void OMLinkKinematics::forward(Manipulator* manipulator, bool *error = false)
{
  KINEMATICS::getPassiveJointAngle(manipulator, error)
  OMKinematicsMethod::forward(manipulator, error);
}

Eigen::VectorXf OMLinkKinematics::geometricInverse(Manipulator* manipulator, Name tool_number, Pose target_pose, float gain) //for basic model
{
  Eigen::VectorXf target_angle_vector(3);
  Eigen::Vector3f control_position; //joint6-joint1
  Eigen::Vector3f tool_relative_position = MANAGER::getComponentRelativePositionToParent(manipulator, tool_number, error);
  Eigen::Vector3f base_position = MANAGER::getComponentPositionToWorld(manipulator, MANAGER::getWorldChildName(manipulator, error), error);
  Eigen::Vector3f temp_vector;

  float target_angle[3];
  float link[3];
  float temp_x;
  float temp_y;

  temp_y = target_pose.position(0)-base_position(0);
  temp_x = target_pose.position(1)-base_position(1);
  target_angle[0] = atan2(temp_y, temp_x);

  control_position(0) = target_pose.position(0) - tool_relative_position(0)*cos(target_angle[0]);
  control_position(1) = target_pose.position(1) - tool_relative_position(0)*sin(target_angle[0]);
  control_position(2) = target_pose.position(2) - tool_relative_position(2);

    // temp_vector = omlink.link_[0].getRelativeJointPosition(1,0);
  temp_vector = MANAGER::getComponentRelativePositionToParent(manipulator, MANAGER::getComponentParentName(manipulator, MANAGER::getComponentParentName(manipulator, MANAGER::getComponentParentName(manipulator, tool_number, error), error), error), error);
  link[0] = temp_vector(2);
    // temp_vector = omlink.link_[1].getRelativeJointPosition(5,1);
  temp_vector = MANAGER::getComponentRelativePositionToParent(manipulator, MANAGER::getComponentParentName(manipulator, MANAGER::getComponentParentName(manipulator, tool_number, error), error), error);
  link[1] = temp_vector(0);
    // temp_vector = omlink.link_[4].getRelativeJointPosition(6,5);
  temp_vector = MANAGER::getComponentRelativePositionToParent(manipulator, MANAGER::getComponentParentName(manipulator, tool_number, error), error);
  link[2] = -temp_vector(0);

  temp_y = control_position(2)-base_position(2);
  temp_x = (control_position(0)-base_position(0))*cos(target_angle[0]);

  target_angle[1] = acos(((temp_x*temp_x+temp_y*temp_y+link[1]*link[1]-link[2]*link[2]))/(2*link[1]*sqrt(temp_x*temp_x+temp_y*temp_y))) + atan2(temp_y, temp_x);
  target_angle[2] = acos((link[1]*link[1]+link[2]*link[2]-(temp_x*temp_x+temp_y*temp_y))/(2*link[1]*link[2])) + target_angle[1];

  target_angle_vector << target_angle[0],
                         target_angle[1],
                         target_angle[2];
  return target_angle_vector;
}



