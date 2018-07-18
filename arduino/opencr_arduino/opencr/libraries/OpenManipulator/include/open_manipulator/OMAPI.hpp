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

#include "OMDebug.hpp"
#include "OMManager.hpp"
#include "OMMath.hpp"
#include "OMKinematics.hpp"
#include "OMDynamixel.hpp"
#include "OMPath.hpp"

osMutexDef(om_mutex);
osMutexId(om_mutex_id);

namespace MUTEX
{
void create(){ om_mutex_id = osMutexCreate(osMutex(om_mutex)); }
void wait(){ osMutexWait(om_mutex_id, osWaitForever); }
void release(){ osMutexRelease(om_mutex_id); }
} // namespace THREAD

namespace OPEN_MANIPULATOR
{
namespace ACTUATOR
{
bool (*setAllJointAngle)(float*) = NULL;
bool (*setJointAngle)(uint8_t, float) = NULL;
float* (*getAngle)() = NULL;
} // namespace ACTUATOR

void linkSetAllJointAnglefunctionToAPI(bool (*fp)(float*)){ ACTUATOR::setAllJointAngle = fp; }
void linkSetJointAnglefunctionToAPI(bool (*fp)(uint8_t, float)){ ACTUATOR::setJointAngle = fp; }
void linkSetGetAnglefunctionToAPI(float* (*fp)()){ ACTUATOR::getAngle = fp; }

// get(position level)


// set(position level)

void Thread_Robot_State(void const *argument)
{
  (void) argument;

  LOG::init();
  MUTEX::create();

  for(;;)
  {
  MUTEX::wait();
    float* angle_ptr = ACTUATOR::getAngle();
  MUTEX::release();
    LOG::INFO("angle : " + String(angle_ptr[0])); 

  osDelay(10);
  }
}
} // namespace OPEN_MANIPULATOR

#endif // OMAPI_HPP_  
   
   
   
   
   
   
  