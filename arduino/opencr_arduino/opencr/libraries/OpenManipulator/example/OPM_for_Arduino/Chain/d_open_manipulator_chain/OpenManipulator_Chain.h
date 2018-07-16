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

/* Authors: Darby Lim */

#ifndef OPEN_MANIPULATOR_CHAIN_H_
#define OPEN_MANIPULATOR_CHAIN_H_

#include "OMManager.hpp"
#include "OMMath.hpp"
#include <Eigen.h>

namespace DYNAMIXEL
{
#define JOINT1_ID 11
#define JOINT2_ID 12
#define JOINT3_ID 13
#define JOINT4_ID 14
}

#define ZERO_VECTOR     Eigen::Vector3f::Zero();
#define IDENTITY_MATRIX Eigen::Matrix3f::Identity(3,3);

// <type "Chain", the number of joint, the number of link, the number of tool> (dof)
Manipulator<1, 4, 4, 1> omChain(4);

static Eigen::Vector3f axisVector(float x, float y, float z)
{
  OMMath math;
  return math.makeEigenVector3(x,y,z);
}

static Eigen::Vector3f relativePosition(float x, float y, float z)
{
  OMMath math;
  return math.makeEigenVector3(x, y, z);
}

static Eigen::Vector3f relativeOrientation(float roll, float pitch, float yaw)
{
  OMMath math;
  return math.makeRotationMatrix(roll, pitch, yaw);
}

static void initJoint()
{
  omChain.joint[0].init(DYNAMIXEL::JOINT1_ID, axisVector(0, 0, 1));
  omChain.joint[1].init(DYNAMIXEL::JOINT2_ID, axisVector(0, 1, 0));
  omChain.joint[2].init(DYNAMIXEL::JOINT3_ID, axisVector(0, 1, 0));
  omCahin.joint[3].init(DYNAMIXEL::JOINT4_ID, axisVector(0, 1, 0));
}

static void initLink()
{
  omChain.base.init(1); //The number of Inner Joint
  omChain.base.setPosition(ZERO_VECTOR);
  omChain.base.setOrientation(IDENTITY_MATRIX);
  omChain.base.setRelativeBasePosition(ZERO_VECTOR);
  omChain.base.setRelativeBaseOrientation(IDENTITY_MATRIX);
  omChain.base.setInnerJoint(0, relativePosition(-170.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));

  omChain.link[0].init(2);
  omChain.link[0].setInnerJoint(0, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  omChain.link[0].setInnerJoint(1, relativePosition(0.012 0.0 0.017), relativeOrientation(0.0, 0.0, 0.0));

  omChain.link[1].init(2);
  omChain.link[0].setInnerJoint(1, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  omChain.link[0].setInnerJoint(2, relativePosition(0.012 0.0 0.017), relativeOrientation(0.0, 0.0, 0.0));
}

void initManipulator()
{
  initJoint();
  initLink();

}

#endif //OPEN_MANIPULATOR_CHAIN_H_