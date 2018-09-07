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
#include "Motion.h"
#include "RemoteController.h"

std::vector<float> goal_position;
Pose goal_pose;

float present_time = 0.0;
float previous_time[3] = {0.0, 0.0, 0.0};

void setup()
{
  Serial.begin(57600);
  DEBUG.begin(57600);
   while (!Serial)
     ;

  connectProcessing();
  //connectRC100();
  
  initManipulator();

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);


  //initThread();
  //startThread();
}

void loop()
{
  present_time = (float)(millis()/1000.0f);

  getData(10);
  if(present_time-previous_time[0] >= LOOP_TIME)
  {
    switchRead();
    setMotion();
    previous_time[0] = (float)(millis()/1000.0f);
  }

  //solve Kinematics
  if(present_time-previous_time[1] >= ROBOT_STATE_UPDATE_TIME)
  {
    updateAllJointAngle();
    chain.forward(chain.getWorldChildName());
    // Pose target_pose = chain.getComponentPoseToWorld(TOOL);
    // DEBUG.print(target_pose.position(0));
    // DEBUG.print(" ");
    // DEBUG.print(target_pose.position(1));
    // DEBUG.print(" ");
    // DEBUG.print(target_pose.position(2));
    // DEBUG.println();
    previous_time[1] = (float)(millis()/1000.0f);
  }

  //Joint Control
  if(present_time-previous_time[2] >= ACTUATOR_CONTROL_TIME)
  {
    chain.setPresentTime((float)(millis()/1000.0f));
    chain.jointControl();
    chain.jointControlForDrawing(TOOL);
#ifdef DEBUGING
    for(int i =0; i < 3; i++)
    {
      DEBUG.print(chain.receiveAllActuatorAngle().at(i));
      DEBUG.print(", ");
    }
    DEBUG.println();
#endif
    previous_time[2] = (float)(millis()/1000.0f);
  }
}

void switchRead()
{
  if(digitalRead(BDPIN_PUSH_SW_1))
    motionStart();
  if(digitalRead(BDPIN_PUSH_SW_2))
    motionStop();
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
      if (processing_flag)
      {
        fromProcessing(get_processing_data);

        tick = millis();
        state = 1;
      }
      else if (rc100_flag)
      {
        fromRC100(get_rc100_data);

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

// /// DON'T TOUCH BELOW CODE///

// namespace THREAD
// {
// osThreadId loop;
// osThreadId robot_state;
// osThreadId actuator_control;
// } // namespace THREAD

// void initThread()
// {
//   MUTEX::create();

//   // define thread
//   osThreadDef(THREAD_NAME_LOOP, Loop, osPriorityNormal, 0, 1024 * 10);
//   osThreadDef(THREAD_NAME_ROBOT_STATE, THREAD::Robot_State, osPriorityNormal, 0, 1024 * 10);
//   osThreadDef(THREAD_NAME_ACTUATOR_CONTROL, THREAD::Actuator_Control, osPriorityNormal, 0, 1024 * 10);

//   // create thread
//   THREAD::loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
//   THREAD::robot_state = osThreadCreate(osThread(THREAD_NAME_ROBOT_STATE), NULL);
//   THREAD::actuator_control = osThreadCreate(osThread(THREAD_NAME_ACTUATOR_CONTROL), NULL);
// }

// void startThread()
// {
//   // start kernel
//   Serial.println("Thread Start");
//   osKernelStart();
// }

// static void Loop(void const *argument)
// {
//   (void)argument;

//   for (;;)
//   {
//     loop();
//     showLedStatus();
//   }
// }

