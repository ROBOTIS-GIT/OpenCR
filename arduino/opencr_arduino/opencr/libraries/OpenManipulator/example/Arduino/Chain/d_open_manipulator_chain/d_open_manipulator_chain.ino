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

#include "Chain.h"
#include "Processing.h"
#include "RemoteController.h"

std::vector<float> goal_position;
Pose goal_pose;

uint8_t seq = 0;

void setup()
{
  Serial.begin(57600);
  while (!Serial)
    ;

  initManipulator();

  connectProcessing();

  // goal_position.push_back(0.0);
  // goal_position.push_back(0.0);
  // goal_position.push_back(0.0);
  // goal_position.push_back(0.0);

  // goal_pose.position = OM_MATH::makeVector3(-0.050, 0.0, 0.203);
  // goal_pose.orientation = Eigen::Matrix3f::Identity();

  initThread();
  startThread();
}

void loop()
{
  fromRC100();

  if (Serial.available())
    fromProcessing(Serial.readStringUntil('\n'));


  // switch(seq)
  // {
  //   case 0:
  //     if (chain.moving() == false)
  //     {
  //       chain.jointMove(CHAIN, goal_position, 1.0f);

  //       seq = 1;
  //     }
  //    break;

  //   case 1:
  //     if (chain.moving() == false)
  //     {
  //       chain.setPose(CHAIN, TOOL, goal_pose, 1.0f);
      
  //       seq = 2;
  //     }
  //    break;

  //   case 2:
  //     if (chain.moving() == false)
  //     {
  //       chain.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.050), 1.0f);

  //       seq = 3;
  //     }
  //    break;

  //   case 3:
  //     chain.toolMove(CHAIN, TOOL, OM_MATH::map(0.060f, 0.020f, 0.070f, 0.907f, -1.13f));
  //     seq = 4;
  //    break;

  //   default:
  //    break;
  // }

  // LOG::INFO("LOOP"); 
  // osDelay(LOOP_TIME * 1000);
}

/// DON'T TOUCH BELOW CODE///

namespace THREAD
{
osThreadId loop;
osThreadId robot_state;
osThreadId actuator_control;
} // namespace THREAD

void initThread()
{
  MUTEX::create();

  // define thread
  osThreadDef(THREAD_NAME_LOOP, Loop, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_ROBOT_STATE, THREAD::Robot_State, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_ACTUATOR_CONTROL, THREAD::Actuator_Control, osPriorityNormal, 0, 1024 * 10);

  // create thread
  THREAD::loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  THREAD::robot_state = osThreadCreate(osThread(THREAD_NAME_ROBOT_STATE), NULL);
  THREAD::actuator_control = osThreadCreate(osThread(THREAD_NAME_ACTUATOR_CONTROL), NULL);
}

void startThread()
{
  // start kernel
  Serial.println("Thread Start");
  osKernelStart();
}

static void Loop(void const *argument)
{
  (void)argument;

  for (;;)
  {
    loop();
    showLedStatus();
  }
}