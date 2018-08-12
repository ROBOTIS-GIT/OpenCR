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

void setup()
{
  Serial.begin(57600);
  while (!Serial);

  initManipulator();

  initThread();
  startThread();
}

void loop()
{
  Serial.println("loop");
  osDelay(500);
}

/// DON'T TOUCH BELOW CODE///

namespace THREAD
{
osThreadId loop;
osThreadId robot_state;
osThreadId motor_control;
}
  char c1 = 'a';
  void *ptr = &c1;

void initThread()
{


  // define thread
  osThreadDef(THREAD_NAME_LOOP, Loop, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_ROBOT_STATE, THREAD::Robot_State, osPriorityNormal, 0, 1024 * 10);
  osThreadDef(THREAD_NAME_MOTOR_CONTROL, THREAD::Motor_Control, osPriorityNormal, 0, 1024 * 10);

  // create thread
  THREAD::loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  THREAD::robot_state = osThreadCreate(osThread(THREAD_NAME_ROBOT_STATE), ptr);
  THREAD::motor_control = osThreadCreate(osThread(THREAD_NAME_MOTOR_CONTROL), NULL);
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