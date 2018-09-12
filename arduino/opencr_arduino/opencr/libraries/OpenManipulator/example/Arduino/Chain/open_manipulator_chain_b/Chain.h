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
#include <OpenManipulator.h>

// User-defined library
#include <OMKinematics.h>
//#include <OMDynamixel.h>
#include "MyDynamixel1.h"
#include "MyDynamixel2.h"
#include <OMDebug.h>

#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define TOOL 5

#define NONE -1

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

#define BAUD_RATE 1000000
//#define DXL_SIZE 5

#define ACTIVE_JOINT_SIZE 4

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator chain;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Chain();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new MY_DYNAMIXEL1::Dynamixel();
#endif /////////////////////////////////////////////
//  OPEN_MANIPULATOR::Path *path = new MY_PATH::Circle();

void initManipulator()
{
  chain.addWorld(WORLD,
                 COMP1);

  chain.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(-0.278, 0.0, 0.017),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     11);

  chain.addComponent(COMP2,
                     COMP1,
                     COMP3,
                     OM_MATH::makeVector3(0.0, 0.0, 0.058),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     12);

  chain.addComponent(COMP3,
                     COMP2,
                     COMP4,
                     OM_MATH::makeVector3(0.024, 0.0, 0.128),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     13);

  chain.addComponent(COMP4,
                     COMP3,
                     TOOL,
                     OM_MATH::makeVector3(0.124, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     14);

  chain.addTool(TOOL,
                COMP4,
                OM_MATH::makeVector3(0.130, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                15,
                1.0f); // Change unit from `meter` to `radian`

  chain.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain.initActuator(actuator);

  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  chain.actuatorInit(p_baud_rate);
  chain.setActuatorControlMode();
  chain.actuatorEnable();
#endif /////////////////////////////////////////////
  chain.initJointTrajectory();
  chain.setControlTime(ACTUATOR_CONTROL_TIME);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain.toolMove(TOOL, 0.0f);
  chain.setAllActiveJointAngle(chain.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  chain.forward(COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  chain.setAllActiveJointAngle(chain.receiveAllActuatorAngle());
#endif
  // Add passive joint function
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

OPEN_MANIPULATOR::OpenManipulator chain2;

OPEN_MANIPULATOR::Kinematics *kinematics2 = new OM_KINEMATICS::Chain();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator2 = new MY_DYNAMIXEL2::Dynamixel();
#endif /////////////////////////////////////////////
//  OPEN_MANIPULATOR::Path *path = new MY_PATH::Circle();

void initManipulator2()
{
  chain2.addWorld(WORLD,
                 COMP1);
  chain2.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(-0.278, 0.0, 0.017),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     16);
  chain2.addComponent(COMP2,
                     COMP1,
                     COMP3,
                     OM_MATH::makeVector3(0.0, 0.0, 0.058),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     17);
  chain2.addComponent(COMP3,
                     COMP2,
                     COMP4,
                     OM_MATH::makeVector3(0.024, 0.0, 0.128),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     18);
  chain2.addComponent(COMP4,
                     COMP3,
                     TOOL,
                     OM_MATH::makeVector3(0.124, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     19);
  chain2.addTool(TOOL,
                COMP4,
                OM_MATH::makeVector3(0.130, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                20,
                1.0f); // Change unit from `meter` to `radian`
  chain2.initKinematics(kinematics2);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain2.initActuator(actuator2);
  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  chain2.actuatorInit(p_baud_rate);
  chain2.setActuatorControlMode();

  chain2.actuatorEnable();
#endif /////////////////////////////////////////////
  chain2.initJointTrajectory();
  chain2.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM ////////////////////////////////////Actuator init
  chain2.toolMove(TOOL, 0.0f);
  chain2.setAllActiveJointAngle(chain2.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  chain2.forward(COMP1);
}

void updateAllJointAngle2()
{
#ifdef PLATFORM
  chain2.setAllActiveJointAngle(chain2.receiveAllActuatorAngle());
#endif
  // Add passive joint function
}


#endif //OPEN_MANIPULATOR_CHAIN_H_
