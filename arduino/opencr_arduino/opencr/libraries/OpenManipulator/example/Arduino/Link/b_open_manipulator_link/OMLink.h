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

/* Authors: Hye-Jong KIM */

#ifndef OMLINK_H_
#define OMLINK_H_

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

//////////////////DebugFlug/////////////////
//#define DEBUGFLUG
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

//////////////motion flug///////////////
bool IK_motion_flug = true;
////////////////////////////////////////////

//////////////suction flug///////////////
bool suction = true;
////////////////////////////////////////////

////////////////using class/////////////////
OPEN_MANIPULATOR::OpenManipulator omlink;

OPEN_MANIPULATOR::Kinematics *kinematics = new OM_KINEMATICS::Link();
#ifdef PLATFORM ////////////////////////////////////Actuator init
OPEN_MANIPULATOR::Actuator *actuator = new OM_DYNAMIXEL::Dynamixel();
#endif /////////////////////////////////////////////
////////////////////////////////////////////

void setPassiveJointAngle()
{
  float joint_angle[3];
  joint_angle[0] = omlink.getComponentJointAngle(JOINT0);
  joint_angle[1] = omlink.getComponentJointAngle(JOINT1);
  joint_angle[2] = omlink.getComponentJointAngle(JOINT2);

  omlink.setComponentJointAngle(JOINT3, (joint_angle[1] - joint_angle[2]));
  omlink.setComponentJointAngle(JOINT4, -M_PI - (joint_angle[1] - joint_angle[2]));
  omlink.setComponentJointAngle(JOINT5, -M_PI - (joint_angle[1] - joint_angle[2]));
  omlink.setComponentJointAngle(JOINT6, -M_PI - joint_angle[2]);
  omlink.setComponentJointAngle(JOINT7, joint_angle[1]);
  omlink.setComponentJointAngle(JOINT8, -(15 * DEG2RAD) - joint_angle[1]);
  omlink.setComponentJointAngle(JOINT9, joint_angle[2] - (195 * DEG2RAD));
  omlink.setComponentJointAngle(JOINT10, (90 * DEG2RAD) - joint_angle[2]);
}

void updateAllJointAngle()
{
#ifdef PLATFORM
  omlink.setAllActiveJointAngle(omlink.receiveAllActuatorAngle());
#endif
  setPassiveJointAngle();
  // Add passive joint function
}

void initOMLink()
{
  //init omlink
  omlink.addWorld(WORLD, JOINT0);
  omlink.addComponent(JOINT0, WORLD, JOINT1,
                           OM_MATH::makeVector3(-0.23867882, 0, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,0,1),
                           1,
                           1);
  omlink.addComponentChild(JOINT0, JOINT2);
  omlink.addComponentChild(JOINT0, JOINT7);
  omlink.addComponent(JOINT1, JOINT0, JOINT5, 
                           OM_MATH::makeVector3(0, 0.022, 0.052),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0),
                           2,
                           -1);
  omlink.addComponent(JOINT2, JOINT0, JOINT3,
                           OM_MATH::makeVector3(0, -0.022, 0.052),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0),
                           3,
                           1);
  omlink.addComponent(JOINT3, JOINT2, JOINT4,
                           OM_MATH::makeVector3(0.050, 0.007, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT4, JOINT3, JOINT5,
                           OM_MATH::makeVector3(0.200, 0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT5, JOINT1, JOINT6,
                           OM_MATH::makeVector3(0.200, -0.016, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT6, JOINT5, SUCTION,
                           OM_MATH::makeVector3(0.200, -0.009, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT7, JOINT0, JOINT8,
                           OM_MATH::makeVector3(-0.04531539, 0.006, 0.07313091),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT8, JOINT7, JOINT9,
                           OM_MATH::makeVector3(0.200, 0.009, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT9, JOINT8, JOINT10,
                           OM_MATH::makeVector3(0.07660444, -0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addComponent(JOINT10, JOINT9, SUCTION,
                           OM_MATH::makeVector3(0.200, -0.006, 0),
                           Matrix3f::Identity(3,3),
                           OM_MATH::makeVector3(0,1,0));
  omlink.addTool(SUCTION, JOINT6,
                      OM_MATH::makeVector3(0.03867882, 0.003, -0.01337315-0.01),
                      Matrix3f::Identity(3,3),
                      4,
                      1);

  omlink.initKinematics(kinematics);
#ifdef PLATFORM ////////////////////////////////////Actuator init
  omlink.initActuator(actuator);
  uint32_t baud_rate = BAUD_RATE;
  void *p_baud_rate = &baud_rate;
  omlink.actuatorInit(p_baud_rate);

  omlink.actuatorEnable();
#endif /////////////////////////////////////////////
  omlink.initJointTrajectory();
  omlink.setControlTime(ACTUATOR_CONTROL_TIME);

#ifdef PLATFORM ////////////////////////////////////Actuator init
  updateAllJointAngle();
#endif /////////////////////////////////////////////
  omlink.forward(); 
  omlink.setPresentTime((float)(millis()/1000.0f));
}

void THREAD::Robot_State(void const *argument)
{
  (void)argument;

  for (;;)
  {
    MUTEX::wait();

    updateAllJointAngle();
    omlink.forward();

#ifdef DEBUGFLUG
    for(int i =0; i < 3; i++)
    {
      DEBUG.print(omlink.receiveAllActuatorAngle().at(i));
      DEBUG.print(", ");
    }
    DEBUG.println();
#endif

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

    omlink.setPresentTime((float)(millis()/1000.0f));
    omlink.jointControl(true);

    MUTEX::release();

    osDelay(ACTUATOR_CONTROL_TIME * 1000);
  }
}

#endif //OMLINK_H_
