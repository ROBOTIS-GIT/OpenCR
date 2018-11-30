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

#include "Link.h"
#include "Processing.h"
#include "Motion.h"
#include "RemoteController.h"

#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35

double present_time = 0.0;
double previous_time = 0.0;

OPEN_MANIPULATOR_LINK omlink;

void setup()
{
  Serial.begin(57600);
  DEBUG.begin(57600);
  while (!Serial)
    ;

  connectProcessing();
  connectRC100();
  switchInit();

  omlink.initManipulator(true);
  RM_LOG::PRINT("OpenManipulator Debugging Port");
}

void loop()
{
  present_time = (double)(millis()/1000.0);

  //get Date 
  getData(10);
  switchRead();
  setMotion(&omlink);

  //Control
  if(present_time-previous_time>= ACTUATOR_CONTROL_TIME)
  {
    omlink.Process((double)(millis()/1000.0));
    sendAngle2Processing(omlink.getManipulator()->getAllActiveJointValue());
    previous_time= (double)(millis()/1000.0);
  }
  
  //osDelay(LOOP_TIME * 1000);
}


////////////////////////getData/////////////////////////////////
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
        fromProcessing(&omlink, get_processing_data);

        tick = millis();
        state = 1;
      }
      else if (rc100_flag)
      {
        fromRC100(&omlink, get_rc100_data);

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
///////////////////////////////////////////////////////////////
////////////////////////switch/////////////////////////////////
void switchInit()
{
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
}

void switchRead()
{
  if(digitalRead(BDPIN_PUSH_SW_1))
  {
    motionStart(&omlink);
  }
  if(digitalRead(BDPIN_PUSH_SW_2))
  {
    motionStop();
  }
}
///////////////////////////////////////////////////////////////