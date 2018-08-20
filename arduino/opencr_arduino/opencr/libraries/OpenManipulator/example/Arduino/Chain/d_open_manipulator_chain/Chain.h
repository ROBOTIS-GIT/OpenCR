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
#include <OMDynamixel.h>

#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.050f

#define CHAIN 100

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
#define DXL_SIZE 5

#define ACTIVE_JOINT_SIZE 4

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator manipulator;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Chain();
#ifdef PLATFORM////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif/////////////////////////////////////////////
//  OPEN_MANIPULATOR::Path *path = new MY_PATH::Circle();

void initManipulator()
{
  manipulator.addManipulator(CHAIN);

  manipulator.addWorld(CHAIN,
                 WORLD,
                 COMP1);

  manipulator.addComponent(CHAIN,
                     COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(-0.278, 0.0, 0.017),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     1);

  manipulator.addComponent(CHAIN,
                     COMP2,
                     COMP1,
                     COMP3,
                     OM_MATH::makeVector3(0.0, 0.0, 0.058),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     2);

  manipulator.addComponent(CHAIN,
                     COMP3,
                     COMP2,
                     COMP4,
                     OM_MATH::makeVector3(0.024, 0.0, 0.128),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     3);

  manipulator.addComponent(CHAIN,
                     COMP4,
                     COMP3,
                     TOOL,
                     OM_MATH::makeVector3(0.124, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Y_AXIS,
                     4);

  manipulator.addTool(CHAIN,
                TOOL,
                COMP4,
                OM_MATH::makeVector3(0.130, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                5,
                1.0f); // Change unit from `meter` to `radian`

  manipulator.initKinematics(kinematics);
#ifdef PLATFORM////////////////////////////////////Actuator init
  manipulator.initActuator(actuator);

  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;

  manipulator.actuatorInit(p_baud_rate);
  manipulator.actuatorEnable();
#endif/////////////////////////////////////////////
  manipulator.initJointTrajectory(CHAIN);
  manipulator.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM////////////////////////////////////Actuator init
  manipulator.setAllActiveJointAngle(CHAIN, manipulator.receiveAllActuatorAngle(CHAIN));
#endif/////////////////////////////////////////////
  manipulator.forward(CHAIN, COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  manipulator.setAllActiveJointAngle(CHAIN, manipulator.receiveAllActuatorAngle(CHAIN));
#endif
  // Add passive joint function
}

void THREAD::Robot_State(void const *argument)
{
  (void)argument;
  // Eigen::Vector3f pose_to_world;

  for (;;)
  {
    MUTEX::wait();

    updateAllJointAngle();    
    manipulator.forward(CHAIN, COMP1);

    MUTEX::release();
    // pose_to_world = manipulator.getComponentPositionToWorld(CHAIN, TOOL);
    // PRINT::VECTOR(pose_to_world);

    osDelay(ROBOT_STATE_UPDATE_TIME * 1000);
  }
}

void THREAD::Actuator_Control(void const *argument)
{
  (void)argument;

  static uint16_t last_time = 0;

  for (;;)
  {
    // uint16_t t = millis();

    // LOG::INFO("Control Time : " + String(t-last_time));
    MUTEX::wait();

    manipulator.jointControl(CHAIN); 
    
    MUTEX::release();
    // last_time = t;

    osDelay(ACTUATOR_CONTROL_TIME * 1000);
  }
}
#endif //OPEN_MANIPULATOR_CHAIN_H_