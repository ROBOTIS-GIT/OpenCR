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

#ifndef OPEN_MANIPULATOR_PLANAR_H_
#define OPEN_MANIPULATOR_PLANAR_H_

// Necessary Library
#include <OpenManipulator.h>

// User-defined Library
#include <OMKinematics.h>
#include <OMDynamixel.h>

// Control Time Set 
#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f

// Define Components
#define WORLD 0
#define COMP1 1
#define COMP2 2
#define COMP3 3
#define COMP4 4
#define COMP5 5
#define COMP6 6
#define COMP7 7
#define COMP8 8
#define COMP9 9
#define TOOL 10

#define NONE -1

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

//-- Dynamixel --/
#define BAUD_RATE 1000000
#define DXL_SIZE 3

#define ACTIVE_JOINT_SIZE 3

#define PLATFORM

OPEN_MANIPULATOR::OpenManipulator planar;

//-- Kinematics Init --//
OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Planar();

//-- Actuator Init --//
#ifdef PLATFORM 
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif 

// OPEN_MANIPULATOR::Path *path = new MY_PATH::Circle();

void setPassiveJointAngle()
{
  float joint_angle[3];
  joint_angle[0] = planar.getComponentJointAngle(COMP1);
  joint_angle[1] = planar.getComponentJointAngle(COMP2);
  joint_angle[2] = planar.getComponentJointAngle(COMP3);

  planar.setComponentJointAngle(COMP4, 0);
  planar.setComponentJointAngle(COMP5, 0);
  planar.setComponentJointAngle(COMP6, 0);
  planar.setComponentJointAngle(COMP7, 0);
  planar.setComponentJointAngle(COMP8, 0);
  planar.setComponentJointAngle(COMP9, 0);
}

void initManipulator()
{
  planar.addWorld(WORLD,
                 COMP1);

  planar.addComponent(COMP1,
                     WORLD,
                     COMP4,
                     OM_MATH::makeVector3(0.0849f, 0.0849f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     1);

 planar.addComponent(COMP2,
                     WORLD,
                     COMP5,
                     OM_MATH::makeVector3(0.0849f, 0.0849f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     2);

  planar.addComponent(COMP3,
                     WORLD,
                     COMP6,
                     OM_MATH::makeVector3(0.0849f, 0.0849f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3),
                     Z_AXIS,
                     3);

  planar.addComponent(COMP4,
                     COMP7,
                     TOOL,
                     OM_MATH::makeVector3(-0.0849f, 0.049f, 0.0),
                     Eigen::Matrix3f::Identity(3, 3));

  planar.addTool(TOOL,    // why defined like this???
                COMP7,
                OM_MATH::makeVector3(0.0, 0.0366, 0.0),
                Eigen::Matrix3f::Identity(3, 3));

  planar.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  planar.initActuator(actuator);
  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  planar.actuatorInit(p_baud_rate);
  planar.actuatorEnable();
#endif /////////////////////////////////////////////
  planar.initJointTrajectory();
  planar.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM ////////////////////////////////////Actuator init    
  std::vector<float> goal_position_;   // rename this var name
  goal_position_.push_back(0.0f);
  goal_position_.push_back(0.0f);
  goal_position_.push_back(0.0f);
  planar.jointMove(goal_position_, 1.0f);

  planar.setAllActiveJointAngle(planar.receiveAllActuatorAngle());
#endif /////////////////////////////////////////////
  // planar.forward(COMP1);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  planar.setAllActiveJointAngle(planar.receiveAllActuatorAngle());
#endif
  // Add passive joint function
}

void THREAD::Robot_State(void const *argument)
{
  (void)argument;

  for (;;)
  {
    MUTEX::wait();

    updateAllJointAngle();
    // planar.forward(COMP1);

    MUTEX::release();

    osDelay(ROBOT_STATE_UPDATE_TIME * 1000);
  }
}

void THREAD::Actuator_Control(void const *argument)
{
  (void)argument;

  for (;;)
  {
    MUTEX::wait();

    planar.jointControl();

    MUTEX::release();

    osDelay(ACTUATOR_CONTROL_TIME * 1000);
  }
}
#endif //OPEN_MANIPULATOR_PLANAR_H_
