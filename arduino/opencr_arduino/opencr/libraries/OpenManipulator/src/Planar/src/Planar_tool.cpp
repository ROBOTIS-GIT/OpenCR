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

#include "../include/open_manipulator_libs/Planar_Tool.h"

using namespace PLANAR_DYNAMIXEL;

/* tool actuator */
void ToolDynamixel::init(uint8_t actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = ToolDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void ToolDynamixel::setMode(const void *arg)
{
  bool result = false;
  const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = ToolDynamixel::setOperatingMode(get_arg_[0]);
    if (result == false)
      return;
  }
  else
  {
    result = ToolDynamixel::writeProfileValue(get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }

  result = ToolDynamixel::setSDKHandler();
  if (result == false)
    return;
}

uint8_t ToolDynamixel::getId()
{
  return dynamixel_.id.at(0);
}

void ToolDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }
}

void ToolDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }
}

bool ToolDynamixel::sendToolActuatorValue(double value)
{
  return ToolDynamixel::writeGoalPosition(value);
}

double ToolDynamixel::receiveToolActuatorValue()
{
  return ToolDynamixel::receiveDynamixelValue();
}

//////////////////////////////////////////////////////////////////////////

bool ToolDynamixel::initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  const char* log = NULL;
  bool result = false;

  dynamixel_.id.push_back(actuator_id);
  dynamixel_.num = 1;

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  uint16_t get_model_number;
  result = dynamixel_workbench_->ping(dynamixel_.id.at(0), &get_model_number, &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
    RM_LOG::ERROR("Please check your Dynamixel ID");
  }
  else
  {
    char str[100];
    sprintf(str, "Tool Dynamixel ID : %d, Model Name :", dynamixel_.id.at(0));
    strcat(str, dynamixel_workbench_->getModelName(dynamixel_.id.at(0)));
    RM_LOG::PRINT(str);
  }

  return true;
}

bool ToolDynamixel::setOperatingMode(STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t effort = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 100;

  if (dynamixel_mode == "position_mode")
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      RM_LOG::ERROR(log);
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    result = dynamixel_workbench_->currentBasedPositionMode(dynamixel_.id.at(0), current, &log);
    if (result == false)
    {
      RM_LOG::ERROR(log);
    }
  }
  else
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      RM_LOG::ERROR(log);
    }
  }

  return true;
}

bool ToolDynamixel::writeProfileValue(STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), char_profile_mode, value, &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  return true;
}

bool ToolDynamixel::setSDKHandler()
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(dynamixel_.id.at(0), "Goal_Position", &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(dynamixel_.id.at(0),
                                                    "Present_Position", 
                                                    &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  return true;
}

bool ToolDynamixel::writeGoalPosition(double radian)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_position = 0;

  goal_position = dynamixel_workbench_->convertRadian2Value(dynamixel_.id.at(0), radian);

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, &goal_position, &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  return true;
}

double ToolDynamixel::receiveDynamixelValue()
{
  bool result = false;
  const char* log = NULL;

  int32_t get_value = 0;
  uint8_t id_array[1] = {dynamixel_.id.at(0)};

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                          id_array,
                                          (uint8_t)1,
                                          &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                            id_array,
                                            (uint8_t)1,
                                            &get_value, 
                                            &log);
  if (result == false)
  {
    RM_LOG::ERROR(log);
  } 

  return dynamixel_workbench_->convertValue2Radian(dynamixel_.id.at(0), get_value);
}
