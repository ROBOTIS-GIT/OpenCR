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

///////////////////////////////////OMChainKinematics/////////////////////////////////////////

OMChainKinematics::OMChainKinematics(){}
OMChainKinematics::~OMChainKinematics(){}

///////////////////////////////////OMScaraKinematics/////////////////////////////////////////

OMScaraKinematics::OMScaraKinematics(){}
OMScaraKinematics::~OMScaraKinematics(){}

///////////////////////////////////OMLinkKinematics/////////////////////////////////////////

OMLinkKinematics::OMLinkKinematics(){}
OMLinkKinematics::~OMLinkKinematics(){}


void OMLinkKinematics::getPassiveJointAngle(Joint* joint)
{

}

void OMLinkKinematics::getBasePose(Base& base, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation)
{
  Pose temp;
  base.setPosition(base_position);
  base.setOrientaion(base_orientation);
  temp.position = base_position + base.inertia_.relative_center_position;
  base.setCenterPosition(temp.position);
}

void OMLinkKinematics::getSinglejointPose(Manipulator* om, int8_t joint_number, int8_t mather_joint_number, int8_t link_number)
{
  Pose temp;
  temp.position = om.joint[mather_joint_number].getPosition() + om.joint[mather_joint_number].getOrientation()*om.link.getRelativeJointPosition(mather_joint_number,joint_number);
  temp.orientation = om.joint[mather_joint_number].getOrientation() * om.link.getRelativeJointOrientation(mather_joint_number,joint_number) * math_.rodriguesRotationMatrix(om.joint[joint_number].getAxis(), om.joint[joint_number].getAngle());
  om.joint[joint_number].setPose(temp);
}

void OMLinkKinematics::getToolPose(Manipulator* om, int8_t tool_number, int8_t mather_joint_number)
{
  // Pose temp;
  // temp.position = om.joint[mather_joint_number].getPosition() + om.joint[mather_joint_number].getOrientation()*om.tool[tool_number].getRelativeToolPosition(mather_joint_number);
  // temp.orientation = om.joint[mather_joint_number].getOrientation() * om.tool[tool_number].getRelativeToolOrientation(mather_joint_number);
  // om.tool[tool_number].setPose(temp);
}


void OMLinkKinematics::forward(Manipulator* omlink, Eigen::Vector3f base_position, Eigen::Matrix3f base_orientation)
{
  Pose temp;

  getPassiveJointAngle(omlink.joint);
  getBasePose(omlink.base, base_position, base_orientation);

  // temp.position = omlink.base.getPosition() + omlink.base.getOrientation()*omlink.base.getRelativeBasePosition(0);
  // temp.orientation = omlink.base.getOrientation() * omlink.base.getRelativeBaseOrientation(0) * math_.rodriguesRotationMatrix(omlink.joint[0].getAxis(), omlink.joint[0].getAngle());
  // omlink.joint[0].setJointPose(temp);

  getSinglejointPose(omlink, 1, 0, 0);
  getSinglejointPose(omlink, 2, 0, 0);
  getSinglejointPose(omlink, 3, 2, 2);
  getSinglejointPose(omlink, 4, 3, 3);
  getSinglejointPose(omlink, 5, 1, 1);
  getSinglejointPose(omlink, 6, 5, 4);
  getSinglejointPose(omlink, 7, 0, 0);
  getSinglejointPose(omlink, 8, 7, 5);
  getSinglejointPose(omlink, 9, 8, 6);
  getSinglejointPose(omlink, 10, 9, 7);

  getToolPose(omlink, 0, 6);


}

void OMLinkKinematics::inverse(Manipulator* omlink)
{

}

///////////////////////////////////OMDeltaKinematics/////////////////////////////////////////

OMDeltaKinematics::OMDeltaKinematics(){}
OMDeltaKinematics::~OMDeltaKinematics(){}

///////////////////////////////////MYKinematics/////////////////////////////////////////

MYKinematics::MYKinematics(){}
MYKinematics::~MYKinematics(){}
