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

#include <OMManager.h>
#include <Eigen.h>

namespace DYNAMIXEL
{
const int8_t JOINT1_ID = 11;
const int8_t JOINT2_ID = 12;
const int8_t JOINT3_ID = 13;
const int8_t JOINT4_ID = 14;

const int32_t BAUD_RATE = 1000000;
const int8_t  DXL_SIZE  = 1;
}

namespace MY_ROBOT
{
const int8_t CHAIN = 1;
const int8_t THE_NUMBER_OF_JOINT = 4;
const int8_t THE_NUMBER_OF_LINK = 4;
const int8_t THE_NUMBER_OF_TOOL = 1;
const int8_t DOF = 4;

Manipulator<CHAIN, THE_NUMBER_OF_JOINT, THE_NUMBER_OF_LINK, THE_NUMBER_OF_TOOL> omChain(DOF);
OMDynamixel<DYNAMIXEL::DXL_SIZE,DYNAMIXEL::BAUD_RATE> omDynamixel;

void initManipulator(void);
}

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

static Eigen::Matrix3f relativeOrientation(float roll, float pitch, float yaw)
{
  OMMath math;
  return math.makeRotationMatrix(roll, pitch, yaw); // radian
}

static void initJoint()
{
  omChain.joint_[0].init(DYNAMIXEL::JOINT1_ID, axisVector(0, 0, 1));
  omChain.joint_[1].init(DYNAMIXEL::JOINT2_ID, axisVector(0, 1, 0));
  omChain.joint_[2].init(DYNAMIXEL::JOINT3_ID, axisVector(0, 1, 0));
  omChain.joint_[3].init(DYNAMIXEL::JOINT4_ID, axisVector(0, 1, 0));
}

static void initLink()
{
  omChain.base_.init(1); //The number of Inner Joint
  omChain.base_.setPosition(ZERO_VECTOR);
  omChain.base_.setOrientation(IDENTITY_MATRIX);
  omChain.base_.setRelativeBasePosition(ZERO_VECTOR);
  omChain.base_.setRelativeBaseOrientation(IDENTITY_MATRIX);
  omChain.base_.setInnerJoint(0, relativePosition(-170.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));

  omChain.link_[0].init(2);
  omChain.link_[0].setInnerJoint(0, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  omChain.link_[0].setInnerJoint(1, relativePosition(0.012, 0.0, 0.017), relativeOrientation(0.0, 0.0, 0.0));

  omChain.link_[1].init(2);
  omChain.link_[1].setInnerJoint(1, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  omChain.link_[1].setInnerJoint(2, relativePosition(0.0, 0.0, 0.058), relativeOrientation(0.0, 0.0, 0.0));

  omChain.link_[2].init(2);
  omChain.link_[2].setInnerJoint(2, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  omChain.link_[2].setInnerJoint(3, relativePosition(0.024, 0.0, 0.128), relativeOrientation(0.0, 0.0, 0.0));
}

static void initTool()
{
  omChain.tool_[0].init(2);
  omChain.tool_[0].setRelativeToolPosition(ZERO_VECTOR);
  omChain.tool_[0].setRelativeToolOrientation(IDENTITY_MATRIX);
  omChain.tool_[0].setInnerJoint(3, relativePosition(0.070, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
}

void initManipulator()
{
  initJoint();
  initLink();
  initTool();
}

#endif //OPEN_MANIPULATOR_CHAIN_H_