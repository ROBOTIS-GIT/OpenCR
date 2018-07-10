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

/* Authors: Hye-Jong KIM, Darby Lim */

#include "../../include/open_manipulator/OMKinematics.h"

///////////////////////////////////OMKinematicsMethod/////////////////////////////////////////

OMKinematicsMethod::OMKinematicsChainMethod(){}

OMKinematicsMethod::~OMKinematicsChainMethod(){}

void OMKinematicsMethod::getBasePose(Base &base, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation)
{
  Pose temp;
  base.setPosition(base_position);
  base.setOrientaion(base_orientation);
  temp.position = base_position + base.inertia_.relative_center_position;
  base.setCenterPosition(temp.position);
}

void OMKinematicsMethod::getBaseJointPose(Manipulator *manipulator, int8_t base_joint_number)
{
  Pose temp;
  temp.position = manipulator.base_.getPosition() + manipulator.base_.getOrientation()*manipulator.base_.getRelativeBaseJointPosition(base_joint_number);
  temp.orientation = manipulator.base_.getOrientation() * manipulator.base_.getRelativeBaseJointOrientation(base_joint_number) * math_.rodriguesRotationMatrix(manipulator.joint_[base_joint_number].getAxis(), manipulator.joint_[base_joint_number].getAngle());
  manipulator.joint[base_joint_number].setJointPose(temp);
}

void OMKinematicsMethod::getSinglejointPose(Manipulator *manipulator, int8_t joint_number, int8_t mather_joint_number, int8_t link_number)
{
  Pose temp;
  temp.position = manipulator.joint_[mather_joint_number].getPosition() + manipulator.joint_[mather_joint_number].getOrientation()*manipulator.link_[link_number].getRelativeJointPosition(mather_joint_number,joint_number);
  temp.orientation = manipulator.joint_[mather_joint_number].getOrientation() * manipulator.link_[link_number].getRelativeJointOrientation(mather_joint_number,joint_number) * math_.rodriguesRotationMatrix(manipulator.joint_[joint_number].getAxis(), manipulator.joint[joint_number].getAngle());
  manipulator.joint_[joint_number].setPose(temp);
}

void OMKinematicsMethod::getToolPose(Manipulator *manipulator, int8_t tool_number, int8_t mather_joint_number)
{
  Pose temp;
  temp.position = manipulator.joint_[mather_joint_number].getPosition() + manipulator.joint_[mather_joint_number].getOrientation()*manipulator.tool_[tool_number].getRelativeToolPosition(mather_joint_number);
  temp.orientation = manipulator.joint_[mather_joint_number].getOrientation() * manipulator.tool_[tool_number].getRelativeToolOrientation(mather_joint_number);
  manipulator.tool_[tool_number].setPose(temp);
}

Eigen::MatrixXf OMKinematicsMethod::jacobian(Manipulator *manipulator, Pose tool_pose, int8_t tool_number)
{
  
  Eigen::MatrixXf jacobian(6,manipulator.getDOF());
  Eigen::Vector3f position_changed    = Eigen::Vector3f::Zero();
  Eigen::Vector3f orientation_changed = Eigen::Vector3f::Zero();
  Eigen::VectorXf pose_changed(6);

  int8_t j = 0;
  for(int8_t i = 0; i < manipulator.getJointSize(); i++)
  {
    if(manipulator.joint_[i].getId() >= 0)
    {
      position_changed = math_.skewSymmetricMatrix(manipulator.joint_[i].getOrientation()*manipulator.joint_[i].getAxis()) * ( manipulator.tool_[tool_number].getPosition() - manipulator.joint_[i].getPosition());
      orientation_changed = manipulator.joint_[i].getOrientation()*manipulator.joint_[i].getAxis();
      
      pose_changed(6) << position_changed(0),
                         position_changed(1),
                         position_changed(2),
                         orientation_changed(0),
                         orientation_changed(1),
                         orientation_changed(2);
      
      jacobian.col(j) = pose_changed;
      j++;
    }
  }
    






}

///////////////////////////////////OMChainKinematics/////////////////////////////////////////

OMChainKinematics::OMChainKinematics(){}
OMChainKinematics::~OMChainKinematics(){}

///////////////////////////////////OMScaraKinematics/////////////////////////////////////////

OMScaraKinematics::OMScaraKinematics(){}
OMScaraKinematics::~OMScaraKinematics(){}

///////////////////////////////////OMLinkKinematics/////////////////////////////////////////

OMLinkKinematics::OMLinkKinematics(){}
OMLinkKinematics::~OMLinkKinematics(){}

void OMLinkKinematics::getPassiveJointAngle(Joint *joint)
{
  // joint[0].setAngle();
  // joint[1].setAngle(); // 0 < joint[1].setAngle < 3PI/4
  // joint[2].setAngle(); // PI/2 && joint[1].setAngle + 45? < joint[2].setAngle < PI
  joint[3].setAngle(joint[1].getAngle()-joint[2].getAngle());
  joint[4].setAngle(-M_PI-(joint[1].getAngle()-joint[2].getAngle()));
  joint[5].setAngle(-M_PI-(joint[1].getAngle()-joint[2].getAngle()));
  joint[6].setAngle((155 * DEG2RAD) - joint[2].getAngle());
  joint[7].setAngle(joint[1].getAngle());
  joint[8].setAngle((25 * DEG2RAD)-joint[1].getAngle());
  joint[9].setAngle(joint[2].getAngle() - (205 * DEG2RAD));
  joint[10].setAngle((90 * DEG2RAD) - joint[2].getAngle());
}

void OMLinkKinematics::forward(Manipulator *omlink, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation)
{
  OMKinematicsMethod::getBasePose(omlink.base_, base_position, base_orientation);
  forward(omlink);
}

void OMLinkKinematics::forward(Manipulator *omlink)
{
  getPassiveJointAngle(omlink.joint_);
  OMKinematicsMethod::getBaseJointPose(omlink,0);
  OMKinematicsMethod::getSinglejointPose(omlink, 1, 0, 0);
  OMKinematicsMethod::getSinglejointPose(omlink, 2, 0, 0);
  OMKinematicsMethod::getSinglejointPose(omlink, 3, 2, 2);
  OMKinematicsMethod::getSinglejointPose(omlink, 4, 3, 3);
  OMKinematicsMethod::getSinglejointPose(omlink, 5, 1, 1);
  OMKinematicsMethod::getSinglejointPose(omlink, 6, 5, 4);
  OMKinematicsMethod::getSinglejointPose(omlink, 7, 0, 0);
  OMKinematicsMethod::getSinglejointPose(omlink, 8, 7, 5);
  OMKinematicsMethod::getSinglejointPose(omlink, 9, 8, 6);
  OMKinematicsMethod::getSinglejointPose(omlink, 10, 9, 7);
  OMKinematicsMethod::getToolPose(omlink, 0, 6);
}

float* OMLinkKinematics::inverse(Manipulator *omlink, Eigen::Vector3f target_position)
{
  Base base;
  Joint joint;
  Link link;


  Eigen::Vector3f target_angle_vector;
  float target_angle[3];
  

  target_angle_vector << 
  
  
  
  target_angle[0] = 





}

///////////////////////////////////OMDeltaKinematics/////////////////////////////////////////

OMDeltaKinematics::OMDeltaKinematics(){}
OMDeltaKinematics::~OMDeltaKinematics(){}

///////////////////////////////////MYKinematics/////////////////////////////////////////

MYKinematics::MYKinematics(){}
MYKinematics::~MYKinematics(){}
