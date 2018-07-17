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

/* Authors: Darby Limm, Hye-Jong KIM */
   
#ifndef OMAPI_HPP_
#define OMAPI_HPP_

#include <RTOS.h>

#include "OMManager.hpp"
#include "OMMath.hpp"
#include "OMKinematics.hpp"
#include "OMDynamixel.hpp"

namespace OPEN_MANIPULATOR
{
template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
OMDynamixel omDynamixel

void initDynamixel()
{
  // OMDynamixel<DYNAMIXEL::DXL_SIZE,DYNAMIXEL::BAUD_RATE> omDynamixel;

    // omDynamixel.init();
  // omDynamixel.setDisable(1);
  // omDynamixel.setPositionControlMode(1);
  // omDynamixel.setEnable(1);

    // float* angle = omDynamixel.getAngle();
  // Serial.print("angle : "); Serial.println(angle[0]);

    static uint32_t loop_cnt = 0;
  
  Serial.print("Loop Cnt : ");
  Serial.println(loop_cnt++);
}

void Thread_Robot_State(void const *argument)
{
  (void) argument;

  // pinMode(13, OUTPUT);

  // static bool index = false;

  for(;;)
  {
    // if (index)
    //   omDynamixel.setAngle(1, 0.0);
    // else
    //   omDynamixel.setAngle(1, 1.0);

    // index = !index;
    // digitalWrite(13, !digitalRead(13));
    static uint32_t robot_state_cnt = 0;
  
    Serial.print("Robot State Cnt : ");
    Serial.println(robot_state_cnt++);
    osDelay(300);
  }
}
}





///////////////////////////////////////get(joint level)//////////////////////////////////////




///////////////////////////////////////set(joint level)//////////////////////////////////////




///////////////////////////////////////get(position level)///////////////////////////////////





///////////////////////////////////////set(position level)///////////////////////////////////






#endif // OMAPI_HPP_  
   
   
   
   
   
   
  