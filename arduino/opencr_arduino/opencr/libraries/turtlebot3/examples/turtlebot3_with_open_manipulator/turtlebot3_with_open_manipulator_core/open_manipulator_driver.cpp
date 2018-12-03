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

/* Authors: Taehun Lim (Darby) */

#include "open_manipulator_driver.h"

OpenManipulatorDriver::OpenManipulatorDriver()
  :torque_state_(false)
{
}

OpenManipulatorDriver::~OpenManipulatorDriver()
{
  closeDynamixel();
}

bool OpenManipulatorDriver::init(uint8_t *joint_id, uint8_t joint_cnt, uint8_t *gripper_id, uint8_t gripper_cnt)
{
  DEBUG_SERIAL.begin(57600);

  const char *log;
  bool result = false;

  result = dxl_wb_.init(DEVICENAME, BAUDRATE, &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to init");
  }
  else
  {
    DEBUG_SERIAL.print("Succeeded to init : ");
    DEBUG_SERIAL.println(BAUDRATE);  
  }

  uint16_t model_number = 0;
  for (uint8_t num = 0; num < joint_cnt; num++)
  {
    result = dxl_wb_.ping(joint_id[num], &model_number, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
      DEBUG_SERIAL.println("Failed to ping");
    }
    else
    {
      DEBUG_SERIAL.println("Succeeded to ping");
      DEBUG_SERIAL.print("id : ");
      DEBUG_SERIAL.print(joint_id[num]);
      DEBUG_SERIAL.print(" model_number : ");
      DEBUG_SERIAL.println(model_number);

      if (dxl_wb_.getProtocolVersion() == 2.0f)
      {
        result = dxl_wb_.torqueOff(joint_id[num], &log);
        if (result == false)
        {
          DEBUG_SERIAL.println(log);
          DEBUG_SERIAL.println("Failed to set torque off");
        }
        else
        {
          DEBUG_SERIAL.println("Succeeded to set torque off");
        }

        result = dxl_wb_.setTimeBasedProfile(joint_id[num], &log);
        if (result == false)
        {
          DEBUG_SERIAL.println(log);
          DEBUG_SERIAL.println("Failed to set velocity based profile mode");
        }
        else
        {
          DEBUG_SERIAL.println("Succeeded to set velocity based profile mode");
        }
      }

      result = dxl_wb_.jointMode(joint_id[num], 0, 0, &log);
      if (result == false)
      {
        DEBUG_SERIAL.println(log);
        DEBUG_SERIAL.println("Failed to change joint mode");
      }
      else
      {
        DEBUG_SERIAL.println("Succeeded to change joint mode");
      }

      joint_.id[num] = joint_id[num];
      joint_.cnt = joint_cnt;
    }
  }

  for (uint8_t num = 0; num < gripper_cnt; num++)
  {
    result = dxl_wb_.ping(gripper_id[num], &model_number, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
      DEBUG_SERIAL.println("Failed to ping");
    }
    else
    {
      DEBUG_SERIAL.println("Succeeded to ping");
      DEBUG_SERIAL.print("id : ");
      DEBUG_SERIAL.print(gripper_id[num]);
      DEBUG_SERIAL.print(" model_number : ");
      DEBUG_SERIAL.println(model_number);

      result = dxl_wb_.currentBasedPositionMode(gripper_id[num], 100, &log);
      if (result == false)
      {
        DEBUG_SERIAL.println(log);
        DEBUG_SERIAL.println("Failed to change gripper mode");
        DEBUG_SERIAL.println("Set joint mode to gripper");
        
        if (dxl_wb_.getProtocolVersion() == 2.0f)
        {
          result = dxl_wb_.torqueOff(joint_id[num], &log);  
          if (result == true) DEBUG_SERIAL.println("Succeeded to set torque off");

          result = dxl_wb_.setTimeBasedProfile(joint_id[num], &log);
          if (result == true) DEBUG_SERIAL.println("Succeeded to set velocity based profile mode");
        }

        result = dxl_wb_.jointMode(gripper_id[num], 0, 0, &log);
        if (result == true) DEBUG_SERIAL.println("Succeeded to change joint mode");
      }
      else
      {
        DEBUG_SERIAL.println("Succeeded to change gripper mode");
      }

      gripper_.id[num] = gripper_id[num];
      gripper_.cnt = gripper_cnt;
    }
  }

  torque_state_ = true;

  result = dxl_wb_.addSyncWriteHandler(joint_.id[0], "Goal_Position", &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to add sync write handler");
  }

  if (dxl_wb_.getProtocolVersion() == 2.0f)
  {
    const uint8_t ADDR_PROFILE_ACCELERATION = 108;
    const uint8_t LEN_PROFILE_ACCELERATION_PROFILE_VELOCITY = 8;

    result = dxl_wb_.addSyncWriteHandler(ADDR_PROFILE_ACCELERATION, LEN_PROFILE_ACCELERATION_PROFILE_VELOCITY, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
      DEBUG_SERIAL.println("Failed to add sync write handler");
    }
  }

  result = dxl_wb_.addSyncReadHandler(joint_.id[0], "Present_Position", &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to add sync read handler");
  }

  double init_joint_position[4] = {0.0, -1.57, 1.37, 0.2258};
  double init_gripper_position[1] = {0.0};

  writeJointProfileControlParam(3.0f);
  writeJointPosition(init_joint_position);
  writeGripperProfileControlParam(0.0f);
  writeGripperPosition(init_gripper_position);

  DEBUG_SERIAL.println("Succeeded to init OpenManipulator Driver");

  return true;
}

void OpenManipulatorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(false);
}

bool OpenManipulatorDriver::setTorque(bool onoff)
{ 
  const char *log;
  bool result = false;

  for (int num = 0; num < joint_.cnt; num++)  
  {
    result = dxl_wb_.torque(joint_.id[num], onoff, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
    }
    else
    {
      DEBUG_SERIAL.println("Succeeded to set torque");
    }
  }

  for (int num = 0; num < gripper_.cnt; num++)  
  {
    result = dxl_wb_.torque(gripper_.id[num], onoff, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
    }
    else
    {
      DEBUG_SERIAL.println("Succeeded to set torque");
    }
  }

  if (result == true)
    torque_state_ = onoff;

  return result;
}

bool OpenManipulatorDriver::getTorqueState()
{
  return torque_state_;
}

bool OpenManipulatorDriver::readPosition(double *get_data)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 0;

  int32_t get_present_position[joint_.cnt + gripper_.cnt];

  if (dxl_wb_.getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_.syncRead(handler_index, &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
      DEBUG_SERIAL.println("Failed to sync read position");
    }

    result = dxl_wb_.getSyncReadData(handler_index, &get_present_position[0], &log);
    if (result == false)
    {
      DEBUG_SERIAL.println(log);
    }

    for (uint8_t num = 0; num < joint_.cnt; num++)
    {
      get_data[num] = dxl_wb_.convertValue2Radian(joint_.id[num], get_present_position[num]);
    }

    for (uint8_t num = 0; num < gripper_.cnt; num++)
    {
      get_data[joint_.cnt + num] = dxl_wb_.convertValue2Radian(gripper_.id[num], get_present_position[joint_.cnt + num]);
    }
  }
  else if (dxl_wb_.getProtocolVersion() == 1.0f)
  {
    for (uint8_t num = 0; num < joint_.cnt; num++)
    {
      result = dxl_wb_.itemRead(joint_.id[num], "Present_Position", &get_present_position[num], &log);
      if (result == false)
      {
        DEBUG_SERIAL.println(log);
        DEBUG_SERIAL.println("Failed to get joint present position");
      }
      else
      {
        get_data[num] = dxl_wb_.convertValue2Radian(joint_.id[num], get_present_position[num]);
      }
    }

    for (uint8_t num = 0; num < gripper_.cnt; num++)
    {
      result = dxl_wb_.itemRead(gripper_.id[num], "Present_Position", &get_present_position[joint_.cnt + num], &log);
      if (result == false)
      {
        DEBUG_SERIAL.println(log);
        DEBUG_SERIAL.println("Failed to get gripper present position");
      }
      else
      {
        get_data[joint_.cnt + num] = dxl_wb_.convertValue2Radian(gripper_.id[num], get_present_position[joint_.cnt + num]);
      }
    }
  }

  return true;
}

bool OpenManipulatorDriver::writeJointProfileControlParam(int32_t set_time)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 1;
  int32_t goal_data[joint_.cnt*2];

  for (uint8_t num = 0; num < joint_.cnt * 2; num = num + 2)
  {
    goal_data[num] = (set_time * 1000) / 4;
    goal_data[num+1] = set_time * 1000;
  }

  result = dxl_wb_.syncWrite(handler_index, joint_.id, joint_.cnt, &goal_data[0], 2, &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to sync write position");
  }

  return true;
}

bool OpenManipulatorDriver::writeJointPosition(double *set_data)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 0;
  int32_t goal_position[joint_.cnt];

  for (int num = 0; num < joint_.cnt; num++)
  {
    goal_position[num] = dxl_wb_.convertRadian2Value(joint_.id[num], set_data[num]);
  }

  result = dxl_wb_.syncWrite(handler_index, joint_.id, joint_.cnt, &goal_position[0], 1, &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to sync write position");
  }

  return true;
}

bool OpenManipulatorDriver::writeGripperProfileControlParam(int32_t set_time)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 1;
  int32_t goal_data[gripper_.cnt*2];

  for (uint8_t num = 0; num < gripper_.cnt; num++)
  {
    goal_data[num] = (set_time * 1000) / 4;
    goal_data[num+1] = set_time * 1000;
  }

  result = dxl_wb_.syncWrite(handler_index, gripper_.id, gripper_.cnt, &goal_data[0], 2, &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to sync write position");
  }

  return true;
}

bool OpenManipulatorDriver::writeGripperPosition(double *set_data)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 0;
  int32_t goal_position[gripper_.cnt];

  for (int num = 0; num < gripper_.cnt; num++)
  {
    goal_position[num] = dxl_wb_.convertRadian2Value(gripper_.id[num], set_data[num]);
  }

  result = dxl_wb_.syncWrite(handler_index, gripper_.id, gripper_.cnt, &goal_position[0], 1, &log);
  if (result == false)
  {
    DEBUG_SERIAL.println(log);
    DEBUG_SERIAL.println("Failed to sync write position");
  }

  return true;
}