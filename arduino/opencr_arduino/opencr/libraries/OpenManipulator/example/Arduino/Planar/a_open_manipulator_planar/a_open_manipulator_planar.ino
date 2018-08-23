/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#include "Planar.h"
#include "Processing.h"
#include "RemoteController.h"

std::vector<float> goal_position;
Pose goal_pose;

uint8_t seq = 0;

void setup()
{
  Serial.begin(57600);
  DEBUG.begin(57600);
  while (!Serial)
    ;

  connectProcessing();
  connectRC100();
  
  initManipulator();

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
  getData(100);
  // switch(seq)
  // {
  //   case 0:
  //     if (manipulator.moving() == false)
  //     {
  //       manipulator.jointMove(CHAIN, goal_position, 1.0f);
  //       seq = 1;
  //     }
  //    break;

  //   case 1:
  //     if (manipulator.moving() == false)
  //     {
  //       manipulator.setPose(CHAIN, TOOL, goal_pose, 1.0f);

  //       seq = 2;
  //     }
  //    break;

  //   case 2:
  //     if (manipulator.moving() == false)
  //     {
  //       manipulator.setMove(CHAIN, TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.050), 1.0f);

  //       seq = 3;
  //     }
  //    break;

  //   case 3:
  //     manipulator.toolMove(CHAIN, TOOL, OM_MATH::map(0.060f, 0.020f, 0.070f, 0.907f, -1.13f));
  //     seq = 4;
  //    break;

  //   default:
  //    break;
  // }

  // LOG::INFO("LOOP");
  osDelay(LOOP_TIME * 1000);
}

//--------------------------------------------------//
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

void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool rc100_flag = false;
  bool processing_flag = false;

  uint16_t get_rc100_data = 0;
  String get_processing_data = "";

  if (availableRC100())
  {
    get_rc100_data = readRC100Data();
    rc100_flag = true;
  }

  if (availableProcessing())
  {
    get_processing_data = readProcessingData();
    processing_flag = true;
  }

  switch (state)
  {
    case 0:
      if (rc100_flag)
      {
        MUTEX::wait();
        fromRC100(get_rc100_data);
        MUTEX::release();

        tick = millis();
        state = 1;
      }
      else if (processing_flag)
      {
        MUTEX::wait();
        fromProcessing(get_processing_data);
        MUTEX::release();

        tick = millis();
        state = 1;
      }
     break;

    case 1:
      if ((millis() - tick) >= wait_time)
      {
        state = 0;
      }
     break;

    default:
      state = 0;
     break;
  }
}