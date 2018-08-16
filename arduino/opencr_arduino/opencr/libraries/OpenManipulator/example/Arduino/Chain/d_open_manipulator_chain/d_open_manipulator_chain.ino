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

#include "OpenManipulator_Chain.h"

std::vector<float> goal_position;
Pose goal_pose;

bool loop_flag = true;

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(57600);
  while (!Serial)
    ;

  initManipulator();

  goal_position.push_back(0.0);
  goal_position.push_back(0.0);
  goal_position.push_back(0.0);
  goal_position.push_back(0.0);

  initThread();
  startThread();
}

void loop()
{
  if (loop_flag)
  {
    // chain.jointMove(CHAIN, goal_position, 2.0f);
    // chain.toolMove(CHAIN, TOOL, false);
    // Serial.println(fmap(0.060, 0.040, 0.070, -0.785, 0.0));
    chain.toolMove(CHAIN, TOOL, fmap(0.060, 0.040, 0.070, -2.09, 0.0));
    loop_flag = false;
  }

  // LOG::INFO("LOOP"); 
  osDelay(LOOP_TIME * 1000);
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
  }
}