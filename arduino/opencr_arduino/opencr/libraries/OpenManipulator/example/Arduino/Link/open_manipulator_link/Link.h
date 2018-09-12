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

/* Authors: Hye-Jong KIM, Darby Lim, Ryan Shim, Yong-Ho Na */

#ifndef link_H_
#define link_H_

// Necessary library
#include <OpenManipulator.h>

// User-defined library
#include <OMKinematics.h>
#include <OMDynamixel.h>
#include <OMDebug.h>

/////////////control time set////////////////
#define ROBOT_STATE_UPDATE_TIME 0.010f
#define ACTUATOR_CONTROL_TIME 0.010f
#define LOOP_TIME 0.010f
////////////////////////////////////////////

/////////////////////NAME/////////////////////
#define WORLD     0
#define JOINT0    1
#define JOINT1    2
#define JOINT2    3
#define JOINT3    4
#define JOINT4    5
#define JOINT5    6
#define JOINT6    7
#define JOINT7    8
#define JOINT8    9
#define JOINT9    10
#define JOINT10   11
#define SUCTION   12
////////////////////////////////////////////

//////////////////Dynamixel/////////////////
#define BAUD_RATE 1000000
#define PLATFORM
////////////////////////////////////////////

//////////////////Debug/////////////////
// #define DEBUGING
////////////////////////////////////////////

//////////////////Move step/////////////////
#define MOVESTEP 0.01
////////////////////////////////////////////

//////////////////Move time/////////////////
#define MOVETIME 1.0
////////////////////////////////////////////

//////////////Suction Pin Num///////////////
#define RELAY_PIN 8
////////////////////////////////////////////

//////////////motion //////////////
bool IK_motion = true;
////////////////////////////////////////////

//////////////suction //////////////
bool suction = true;
////////////////////////////////////////////

////////////////using class/////////////////
OPEN_MANIPULATOR::OpenManipulator Link;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Link();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif /////////////////////////////////////////////
////////////////////////////////////////////

void setPassiveJointAngle()
{
  float joint_angle[3];
  joint_angle[0] = Link.getComponentJointAngle(JOINT0);
  joint_angle[1] = Link.getComponentJointAngle(JOINT1);
  joint_angle[2] = Link.getComponentJointAngle(JOINT2);

  Link.setComponentJointAngle(JOINT3, (joint_angle[1] - joint_angle[2]));
  Link.setComponentJointAngle(JOINT4, -M_PI - (joint_angle[1] - joint_angle[2]));
  Link.setComponentJointAngle(JOINT5, -M_PI - (joint_angle[1] - joint_angle[2]));
  Link.setComponentJointAngle(JOINT6, -M_PI - joint_angle[2]);
  Link.setComponentJointAngle(JOINT7, joint_angle[1]);
  Link.setComponentJointAngle(JOINT8, -(15 * DEG2RAD) - joint_angle[1]);
  Link.setComponentJointAngle(JOINT9, joint_angle[2] - (195 * DEG2RAD));
  Link.setComponentJointAngle(JOINT10, (90 * DEG2RAD) - joint_angle[2]);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  Link.setAllActiveJointAngle(Link.receiveAllActuatorAngle());
#endif
  setPassiveJointAngle();
  // Add passive joint function
}

void initManipulator()
{
  //init Link
  Link.addWorld(WORLD, JOINT0);
  Link.addComponent(JOINT0, WORLD, JOINT1,
                           OM_MATH::makeVector3(-0.23867882, 0, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,0,1),
                           1,
                           1);
  Link.addComponentChild(JOINT0, JOINT2);
  Link.addComponentChild(JOINT0, JOINT7);
  Link.addComponent(JOINT1, JOINT0, JOINT5, 
                           OM_MATH::makeVector3(0, 0.022, 0.052),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0),
                           2,
                           -1);
  Link.addComponent(JOINT2, JOINT0, JOINT3,
                           OM_MATH::makeVector3(0, -0.022, 0.052),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0),
                           3,
                           1);
  Link.addComponent(JOINT3, JOINT2, JOINT4,
                           OM_MATH::makeVector3(0.050, 0.007, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT4, JOINT3, JOINT5,
                           OM_MATH::makeVector3(0.200, 0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT5, JOINT1, JOINT6,
                           OM_MATH::makeVector3(0.200, -0.016, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT6, JOINT5, SUCTION,
                           OM_MATH::makeVector3(0.200, -0.009, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT7, JOINT0, JOINT8,
                           OM_MATH::makeVector3(-0.04531539, 0.006, 0.07313091),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT8, JOINT7, JOINT9,
                           OM_MATH::makeVector3(0.200, 0.009, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT9, JOINT8, JOINT10,
                           OM_MATH::makeVector3(0.07660444, -0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addComponent(JOINT10, JOINT9, SUCTION,
                           OM_MATH::makeVector3(0.200, -0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  Link.addTool(SUCTION, JOINT6,
                      OM_MATH::makeVector3(0.03867882, 0.003, -0.01337315-0.01),
                      Matrix3f::Identity(3,3),
                      4,
                      1);

  Link.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  Link.initActuator(actuator);
  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  Link.actuatorInit(p_baud_rate);

  Link.actuatorEnable();
#endif /////////////////////////////////////////////
  Link.initJointTrajectory();
  Link.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM ////////////////////////////////////Actuator init
  updateAllJointAngle();
#endif /////////////////////////////////////////////
  Link.forward(); 
  Link.setPresentTime((float)(millis()/1000.0f));
}

#endif //link_H_
