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

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <RobotisManipulator.h>

#define BDPIN_RELAY         8
#define BDPIN_PUMP_MOTOR    12

namespace actuator
{
class GripperVacuum : public robotis_manipulator::ToolActuator
{
 private:
  int8_t actuator_id_;
  ActuatorValue tool_value_;

 public:
  GripperVacuum() {}
  virtual ~GripperVacuum() {}

  virtual void init(uint8_t actuator_id, const void *arg)
  {
    actuator_id_ = actuator_id;
    tool_value_.position = 0.0;
    pinMode(BDPIN_RELAY, OUTPUT);
    pinMode(BDPIN_PUMP_MOTOR, OUTPUT);
  }
  virtual void setMode(const void *arg){}
  virtual uint8_t getId()
  {
    return actuator_id_;
  }

  virtual void enable(){}
  virtual void disable(){}

  virtual bool sendToolActuatorValue(ActuatorValue value)
  {
    if(value.position == 0.0)
    {
      tool_value_.position = 0.0;
      digitalWrite(BDPIN_RELAY, LOW);
      digitalWrite(BDPIN_PUMP_MOTOR, LOW);
    }
    else if(value.position == 1.0)
    {
      tool_value_.position = 1.0;
      digitalWrite(BDPIN_RELAY, HIGH);
      digitalWrite(BDPIN_PUMP_MOTOR, HIGH);
    }
    return true;
  }
  virtual ActuatorValue receiveToolActuatorValue()
  {
    return tool_value_;
  }

////////////////////////////////////////////////////////////////
};

} // namespace actuator
#endif // ACTUATOR_H_




