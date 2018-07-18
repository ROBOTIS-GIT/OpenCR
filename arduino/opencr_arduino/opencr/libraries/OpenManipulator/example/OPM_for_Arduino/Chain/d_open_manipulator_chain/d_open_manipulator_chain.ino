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

#include <RTOS.h>
#include "OpenManipulator_Chain.h"

#define ACTUATOR_ENABLE  true
#define ACTUATOR_DISABLE false

namespace THREAD
{
osThreadId loop;
osThreadId robot_state;
}

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  MY_ROBOT::initManipulator();
  MY_ROBOT::initActuator(ACTUATOR_ENABLE);

  OPEN_MANIPULATOR::linkSetAllJointAnglefunctionToAPI(MY_ROBOT::setAllJointAngle);
  OPEN_MANIPULATOR::linkSetJointAnglefunctionToAPI(MY_ROBOT::setJointAngle);
  OPEN_MANIPULATOR::linkSetGetAnglefunctionToAPI(MY_ROBOT::getAngle);

  initThread();
  startThread();
}

void loop() 
{
  static int loop_cnt = 0;
  if (loop_cnt%10 == 0)
  {
    MUTEX::wait();
    MY_ROBOT::setJointAngle(1, -2.0);
    MUTEX::release();
    Serial.println("order -2.0");
  }
  else if (loop_cnt%10 == 5)
  {
    MUTEX::wait();
    MY_ROBOT::setJointAngle(1, 2.0);
    MUTEX::release();
    Serial.println("order 2.0");
  }
  loop_cnt++;
  osDelay(100);    
}

/// DON'T TOUCH ///

void initThread()
{
  // define thread
  osThreadDef(THREAD_NAME_LOOP,         Thread_Loop,                           osPriorityNormal, 0, 1024*10);
  osThreadDef(THREAD_NAME_ROBOT_STATE,  OPEN_MANIPULATOR::Thread_Robot_State,  osPriorityNormal, 0, 1024*20);

  // create thread
  THREAD::loop         = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  THREAD::robot_state  = osThreadCreate(osThread(THREAD_NAME_ROBOT_STATE), NULL);
}

void startThread()
{
  // start kernel
  Serial.println("Thread Start");
  osKernelStart();
}

static void Thread_Loop(void const *argument)
{
  (void) argument;

  for(;;)
  {
    loop();
  }
}