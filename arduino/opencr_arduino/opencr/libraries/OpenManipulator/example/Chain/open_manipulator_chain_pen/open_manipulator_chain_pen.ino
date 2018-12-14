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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "OpenManipulatorPen.h"
#include "OpenManipulatorPenMotion.h"
#include "Processing.h"

OPEN_MANIPULATOR_PEN open_manipulator;
double present_time = 0.0;
double previous_time = 0.0;

void setup()
{
  Serial.begin(57600);
  DEBUG.begin(57600);

  connectProcessing();
  switchInit();
  
  open_manipulator.initManipulator(true);
  RM_LOG::PRINT("OpenManipulator Debugging Port");
}

void loop()
{
  present_time = (float)(millis()/1000.0f);
  getData(100);
  switchRead(&open_manipulator);
  playMotion(&open_manipulator);
  //RM_LOG::PRINT("x ", open_manipulator.getPose("pen").position(0));
  //RM_LOG::PRINTLN(" y ", open_manipulator.getPose("pen").position(1));

  if(present_time-previous_time >= CONTROL_TIME)
  {
    open_manipulator.openManipulatorProcess(millis()/1000.0);
    previous_time = (float)(millis()/1000.0f);
    sendValueToProcessing(&open_manipulator);
  }
}

void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool processing_flag = false;
  String get_processing_data = "";

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
        fromProcessing(&open_manipulator, get_processing_data);
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