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

#ifndef OPEN_MANIPULATOR_SCARA_H_
#define OPEN_MANIPULATOR_SCARA_H_

// Necessary library
#include <OpenManipulator.h>

// User-defined library
#include <OMKinematics.h>
#include <OMDynamixel.h>

#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f

#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define TOOL 4

#define NONE -1

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

#define BAUD_RATE 1000000
#define DXL_SIZE 4

#define ACTIVE_JOINT_SIZE 3

#define CIRCLE 11
#define RHOMBUS 12
#define HEART 13

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator SCARA;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::SCARA();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif /////////////////////////////////////////////
OPEN_MANIPULATOR::Draw *circle = new OM_PATH::Circle();
OPEN_MANIPULATOR::Draw *rhombus = new OM_PATH::Rhombus();
OPEN_MANIPULATOR::Draw *heart = new OM_PATH::Heart();

void initManipulator()
{
  SCARA.addWorld(WORLD,
                 COMP1);

  SCARA.addComponent(COMP1,
                     WORLD,
                     COMP2,
                     OM_MATH::makeVector3(-0.241, 0.0, 0.057),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     1);

  SCARA.addComponent(COMP2,
                     COMP1,
                     COMP3,
                     OM_MATH::makeVector3(0.067, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     2);

  SCARA.addComponent(COMP3,
                     COMP2,
                     TOOL,
                     OM_MATH::makeVector3(0.067, 0.0, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     3);

  SCARA.addTool(TOOL,
                COMP3,
                OM_MATH::makeVector3(0.107, 0.0, 0.0),
                Eigen::Matrix3f::Identity(3, 3),
                4,
                1.0f); // Change unit from `meter` to `radian`

  SCARA.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  SCARA.initActuator(actuator);

  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;

  SCARA.actuatorInit(p_baud_rate);
  // SCARA.setActuatorControlMode();

  SCARA.actuatorEnable();
#endif /////////////////////////////////////////////
  SCARA.initJointTrajectory();
  SCARA.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM ////////////////////////////////////Actuator init
  SCARA.toolMove(TOOL, 0.0f);
  SCARA.setAllActiveJointAngle(SCARA.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  SCARA.forward(COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  SCARA.setAllActiveJointAngle(SCARA.receiveAllActuatorAngle());
#endif
  // Add passive joint function
}

// void THREAD::Robot_State(void const *argument)
// {
//   (void)argument;

//   for (;;)
//   {
//     MUTEX::wait();

//     updateAllJointAngle();
//     SCARA.forward(COMP1);

//     MUTEX::release();

//     osDelay(ROBOT_STATE_UPDATE_TIME * 1000);
//   }
// }

// void THREAD::Actuator_Control(void const *argument)
// {
//   (void)argument;

//   for (;;)
//   {
//     MUTEX::wait();

//     SCARA.jointControl();
//     SCARA.jointControlForDrawing(TOOL);

//     MUTEX::release();

//     osDelay(ACTUATOR_CONTROL_TIME * 1000);
//   }
// }
#endif //OPEN_MANIPULATOR_SCARA_H_
