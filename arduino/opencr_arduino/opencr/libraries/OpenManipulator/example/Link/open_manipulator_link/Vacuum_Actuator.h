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

#ifndef VACUUM_ACTUATOR_H_
#define VACUUM_ACTUATOR_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif
#include "Arduino.h"


class VacuumModule : public ROBOTIS_MANIPULATOR::ToolActuator
{
 private:
  uint8_t relay_pin_;
  int8_t id_;
  bool suction_flag_;

 public:
  VacuumModule(){}
  virtual ~VacuumModule(){}

  virtual void init(uint8_t actuator_id, const void *arg)
  {
    id_ = actuator_id;
   
    uint8_t *get_arg = (uint8_t *)arg;
    relay_pin_ = get_arg[0];

    vacuumInit();
  }

  virtual void setMode(const void *arg){}

  virtual uint8_t getId()
  {
    return id_;
  }

  virtual void enable(){}
  virtual void disable(){}

  virtual bool sendToolActuatorValue(double value)
  {
    if(value > 0)
    {
      suction_flag_ = true;
      vacuumOn();
    }  
    else
    {
      suction_flag_ = false;
      vacuumOff();
    }
  }

  virtual double receiveToolActuatorValue()
  { 
    if(suction_flag_)
    {
      return 1.0;
    }
    else
    {
      return -1.0;
    }
  }

  void vacuumInit()
  {
    pinMode(relay_pin_, OUTPUT);
  }

  void vacuumOn()
  {
    digitalWrite(relay_pin_, HIGH);
  }

  void vacuumOff()
  {
    digitalWrite(relay_pin_, LOW);
  }
};




#endif //LINK_VACUUM_H_