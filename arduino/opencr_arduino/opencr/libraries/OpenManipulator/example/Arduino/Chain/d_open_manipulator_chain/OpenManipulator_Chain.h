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

// Necessary library
#include <Eigen.h>
#include <OpenManipulator.h>

// User-defined library
#include <OMKinematics.h>
#include <OMDynamixel.h>
#include <OMPath.h>

#define INFO(x) Serial.println(x)

namespace DYNAMIXEL
{
const int8_t ID1 = 1;
const int8_t ID2 = 2;
const int8_t ID3 = 3;
const int8_t ID4 = 4;

const int32_t BAUD_RATE = 57600;
const int8_t  SIZE  = 1;
} // namespace DYNAMIXEL

const int8_t THE_NUMBER_OF_JOINT = 4;
const int8_t THE_NUMBER_OF_LINK = 4;
const int8_t THE_NUMBER_OF_TOOL = 1;
const int8_t DOF = 4;

// Manipulator<THE_NUMBER_OF_JOINT, THE_NUMBER_OF_LINK, THE_NUMBER_OF_TOOL> omChain(DOF);
OMDynamixel<DYNAMIXEL::SIZE, DYNAMIXEL::BAUD_RATE> omDynamixel;

#if 0
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
  MY_ROBOT::omChain.joint_[0].init(DYNAMIXEL::ACTUATOR_ID1, axisVector(0, 0, 1));
  MY_ROBOT::omChain.joint_[1].init(DYNAMIXEL::ACTUATOR_ID2, axisVector(0, 1, 0));
  MY_ROBOT::omChain.joint_[2].init(DYNAMIXEL::ACTUATOR_ID3, axisVector(0, 1, 0));
  MY_ROBOT::omChain.joint_[3].init(DYNAMIXEL::ACTUATOR_ID4, axisVector(0, 1, 0));
}

static void initLink()
{
  MY_ROBOT::omChain.base_.init(1); //The number of Inner Joint
  MY_ROBOT::omChain.base_.setPosition(ZERO_VECTOR);
  MY_ROBOT::omChain.base_.setOrientation(IDENTITY_MATRIX);
  MY_ROBOT::omChain.base_.setRelativeBasePosition(ZERO_VECTOR);
  MY_ROBOT::omChain.base_.setRelativeBaseOrientation(IDENTITY_MATRIX);
  MY_ROBOT::omChain.base_.setInnerJoint(0, relativePosition(-170.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));

  MY_ROBOT::omChain.link_[0].init(2);
  MY_ROBOT::omChain.link_[0].setInnerJoint(0, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  MY_ROBOT::omChain.link_[0].setInnerJoint(1, relativePosition(0.012, 0.0, 0.017), relativeOrientation(0.0, 0.0, 0.0));

  MY_ROBOT::omChain.link_[1].init(2);
  MY_ROBOT::omChain.link_[1].setInnerJoint(1, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  MY_ROBOT::omChain.link_[1].setInnerJoint(2, relativePosition(0.0, 0.0, 0.058), relativeOrientation(0.0, 0.0, 0.0));

  MY_ROBOT::omChain.link_[2].init(2);
  MY_ROBOT::omChain.link_[2].setInnerJoint(2, relativePosition(0.0, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
  MY_ROBOT::omChain.link_[2].setInnerJoint(3, relativePosition(0.024, 0.0, 0.128), relativeOrientation(0.0, 0.0, 0.0));
}

static void initTool()
{
  MY_ROBOT::omChain.tool_[0].init(2);
  MY_ROBOT::omChain.tool_[0].setRelativeToolPosition(ZERO_VECTOR);
  MY_ROBOT::omChain.tool_[0].setRelativeToolOrientation(IDENTITY_MATRIX);
  MY_ROBOT::omChain.tool_[0].setInnerJoint(3, relativePosition(0.070, 0.0, 0.0), relativeOrientation(0.0, 0.0, 0.0));
}
#endif

bool initManipulator()
{
  // Add configuration for your robot
  // initJoint();
  // initLink();
  // initTool();

  INFO("Success to load manipulator");

  return true;
}

bool initActuator(bool enable)
{
  // Add configuration for actuators(DC motor, servo, Dynamixel) such as port, motor speed, etc
  bool isOK = false;
  
  isOK = omDynamixel.init();
  isOK == true ? INFO("Success to load Dynamixels") : INFO("Failed to load Dynamixels");

  isOK = omDynamixel.setPositionControlMode(DYNAMIXEL::ID1);
  isOK == true ? INFO("Success to set Position Control Mode to Dynamixels") : INFO("Failed to set Position Control Mode to Dynamixels");

  if (enable) omDynamixel.enableAllDynamixel();
  else        omDynamixel.disableAllDynamixel();

  return isOK;  
}

bool setAllJointAngle(float *radian)
{
  bool isOK = omDynamixel.setAngle(radian);
  return isOK;
}

bool setJointAngle(uint8_t actuator_id, float radian)
{
  bool isOK = omDynamixel.setAngle(actuator_id, radian);
  return isOK;
}

float* getAngle()
{
  return omDynamixel.getAngle();
}

void initKinematics()
{
  INFO("Success to init kinematics lib");
  return;
}

void forward()
{
  INFO("forward");
  return;
}

void inverse()
{
  INFO("inverse");
  return;
}

void initPath()
{
  INFO("Success to init path lib");
  return;
}

void line()
{
  INFO("line path");
  return;
}

void arc()
{
  INFO("arc path");
  return;
}

void heart()
{
  INFO("heart path");
  return;
}

#endif //OPEN_MANIPULATOR_CHAIN_H_