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

#include "../../include/robotis_manipulator/robotis_manipulator_manager.h"

using namespace robotis_manipulator;

bool JointActuator::findId(uint8_t actuator_id)
{
  std::vector<uint8_t> id = getId();
  for(uint32_t index = 0; index < id.size(); index++)
  {
    if(id.at(index) == actuator_id)
      return true;
  }
  return false;
}

bool JointActuator::getEnabledState()
{
  return enabled_state_;
}

bool ToolActuator::findId(uint8_t actuator_id)
{
  if(getId() == actuator_id)
  {
    return true;
  }
  return false;
}

bool ToolActuator::getEnabledState()
{
  return enabled_state_;
}