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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"

DynamixelWorkbench::DynamixelWorkbench()
{

}

DynamixelWorkbench::~DynamixelWorkbench()
{

}

bool DynamixelWorkbench::begin(const char* device_name, uint32_t baud_rate)
{
  bool isOK = false;

  isOK = driver_.init(device_name, baud_rate);

  return isOK;
}

bool DynamixelWorkbench::scan(uint8_t *get_id, uint8_t *get_id_num, uint8_t range)
{
  bool isOK = false;

  isOK = driver_.scan(get_id, get_id_num, range);

  return isOK;
}

bool DynamixelWorkbench::ping(uint8_t id, uint16_t *get_model_number)
{
  bool isOK = false;

  isOK = driver_.ping(id, get_model_number);

  return isOK;
}

bool DynamixelWorkbench::reboot(uint8_t id)
{
  bool isOK = false;

  isOK = driver_.reboot(id);

  return isOK;
}

bool DynamixelWorkbench::reset(uint8_t id)
{
  bool isOK = false;

  isOK = driver_.reset(id);

  return isOK;
}

bool DynamixelWorkbench::setID(uint8_t id, uint8_t new_id)
{
  bool comm_result = false;

  torque(id, false);

  comm_result = driver_.writeRegister(id, "ID", new_id);

  millis(1000);

  return comm_result;
}

bool DynamixelWorkbench::setBaud(uint8_t id, uint32_t new_baud)
{
  bool comm_result = false;

  torque(id, false);

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (new_baud == 9600)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 207);
    else if (new_baud == 19200)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 103);
    else if (new_baud == 57600)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 34);
    else if (new_baud == 115200)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 16);
    else if (new_baud == 200000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 9);
    else if (new_baud == 250000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 7);
    else if (new_baud == 400000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 4);
    else if (new_baud == 500000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 3);
    else if (new_baud == 1000000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 1);
    else
      comm_result = driver_.writeRegister(id, "Baud_Rate", 34);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (new_baud == 9600)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 0);
    else if (new_baud == 57600)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 1);
    else if (new_baud == 115200)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 2);
    else if (new_baud == 1000000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 3);
    else if (new_baud == 2000000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 4);
    else if (new_baud == 3000000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 5);
    else if (new_baud == 4000000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 6);
    else if (new_baud == 4500000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 7);
    else if (new_baud == 10500000)
      comm_result = driver_.writeRegister(id, "Baud_Rate", 8);
    else
      comm_result = driver_.writeRegister(id, "Baud_Rate", 1);
  }
  millis(2000);

  return comm_result;
}

bool DynamixelWorkbench::setPacketHandler(float protocol_version)
{
  return driver_.setPacketHandler(protocol_version);
}

float DynamixelWorkbench::getProtocolVersion()
{
  return driver_.getProtocolVersion();
}

char* DynamixelWorkbench::getModelName(uint8_t id)
{
  return driver_.getModelName(id);
}

bool DynamixelWorkbench::ledOn(uint8_t id)
{
  bool comm_result = false;

  if (!strncmp(getModelName(id), "PRO", strlen("PRO")))
    comm_result = driver_.writeRegister(id, "LED_RED", 1);
  else
    comm_result = driver_.writeRegister(id, "LED", 1);

  return comm_result;
}

bool DynamixelWorkbench::ledOff(uint8_t id)
{
  bool comm_result = false;

  if (!strncmp(getModelName(id), "PRO", strlen("PRO")))
    comm_result = driver_.writeRegister(id, "LED_RED", 0);
  else
    comm_result = driver_.writeRegister(id, "LED", 0);

  return comm_result;
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));

  comm_result = torque(id, false);

  comm_result = setPositionControlMode(id);

  comm_result = torque(id, true);

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH")))
    {
      comm_result = driver_.writeRegister(id, "Profile_Acceleration", acc);
      comm_result = driver_.writeRegister(id, "Profile_Velocity", vel);
    }
    else
    {
      comm_result = driver_.writeRegister(id, "Moving_Speed", vel);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6) || !strncmp(dxl_, "PRO", 3))
    {
      comm_result = driver_.writeRegister(id, "Moving_Speed", vel);
    }
    else
    {
      comm_result = driver_.writeRegister(id, "Profile_Acceleration", acc);
      comm_result = driver_.writeRegister(id, "Profile_Velocity", vel);
    }
  }

  return comm_result;
}

bool DynamixelWorkbench::wheelMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));

  comm_result = torque(id, false);

  comm_result = setVelocityControlMode(id);

  comm_result = torque(id, true);

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH")))
    {
      comm_result = driver_.writeRegister(id, "Profile_Acceleration", acc);
      comm_result = driver_.writeRegister(id, "Profile_Velocity", vel);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0 && (strncmp(dxl_, "PRO", 3) != 0))
  {   
    comm_result = driver_.writeRegister(id, "Profile_Acceleration", acc);
    comm_result = driver_.writeRegister(id, "Profile_Velocity", vel);
  }

  return comm_result;
}

bool DynamixelWorkbench::currentMode(uint8_t id, uint8_t cur)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));
  
  comm_result = torque(id, false);

  comm_result = setCurrentControlMode(id);

  comm_result = torque(id, true);

  if (!strncmp(dxl_, "X", 1)                         ||
      !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) )
  {   
    comm_result = driver_.writeRegister(id, "Goal_Current", cur);
  }

  return comm_result;
}

bool DynamixelWorkbench::goalPosition(uint8_t id, int32_t goal)
{
  bool comm_result = false;
  
  comm_result = driver_.writeRegister(id, "Goal_Position", goal);

  return comm_result;
}

bool DynamixelWorkbench::goalSpeed(uint8_t id, int32_t goal)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH")))
    {
      comm_result = driver_.writeRegister(id, "Goal_Velocity", goal);
    }
    else
    {
      if (goal < 0)
      {
        goal = (-1) * goal;
        goal |= 1024;
      }
      comm_result = driver_.writeRegister(id, "Moving_Speed", goal);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      if (goal < 0)
      {
        goal = (-1) * goal;
        goal |= 1024;
      }
      comm_result = driver_.writeRegister(id, "Moving_Speed", goal);
    }
    else
      comm_result = driver_.writeRegister(id, "Goal_Velocity", goal);
  }

  return comm_result;
}

bool DynamixelWorkbench::itemWrite(uint8_t id, const char* item_name, int32_t value)
{
  bool comm_result = false;

  comm_result = driver_.writeRegister(id, item_name, value);

  return comm_result;
}

bool DynamixelWorkbench::itemWrite(uint8_t id, uint16_t addr, uint8_t length, int32_t data)
{
  bool comm_result = false;

  comm_result = driver_.writeRegister(id, addr, length, data);

  return comm_result;
}

bool DynamixelWorkbench::syncWrite(const char *item_name, int32_t* value)
{
  bool isOK = false;

  isOK =  driver_.syncWrite(item_name, value);

  return isOK;
}

bool DynamixelWorkbench::syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data)
{
  bool isOK = false;

  isOK =  driver_.syncWrite(id, id_num, item_name, data);

  return isOK;
}

bool DynamixelWorkbench::bulkWrite()
{
  bool isOK = false;

  isOK = driver_.bulkWrite();

  return isOK;
}

int32_t DynamixelWorkbench::itemRead(uint8_t id, const char* item_name)
{
  static int32_t data = 0;

  if (driver_.readRegister(id, item_name, &data))
    return data;
}

int32_t  DynamixelWorkbench::itemRead(uint8_t id, uint16_t addr, uint8_t length)
{
  static int32_t data = 0;

  if (driver_.readRegister(id, addr, length, &data))
    return data; 
}

int32_t* DynamixelWorkbench::syncRead(const char *item_name)
{
  static int32_t data[16];
  if (driver_.syncRead(item_name, data))
    return data;
}

int32_t DynamixelWorkbench::bulkRead(uint8_t id, const char* item_name)
{
  static int32_t data;
  if (driver_.bulkRead(id, item_name, &data))
    return data;
}

void DynamixelWorkbench::addSyncWrite(const char* item_name)
{
  driver_.addSyncWrite(item_name);
}

void DynamixelWorkbench::addSyncRead(const char* item_name)
{
  driver_.addSyncRead(item_name);
}

void DynamixelWorkbench::initBulkWrite()
{
  driver_.initBulkWrite();
}

void DynamixelWorkbench::initBulkRead()
{
  driver_.initBulkRead();
}

bool DynamixelWorkbench::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
{
  bool isOK = false;

  isOK = driver_.addBulkWriteParam(id, item_name, data);

  return isOK;
}

bool DynamixelWorkbench::addBulkReadParam(uint8_t id, const char *item_name)
{
  bool isOK = false;

  isOK = driver_.addBulkReadParam(id, item_name);

  return isOK;
}

bool DynamixelWorkbench::setBulkRead()
{
  bool isOK = false;

  isOK = driver_.sendBulkReadPacket();

  return isOK;
}

int32_t DynamixelWorkbench::convertRadian2Value(uint8_t id, float radian)
{
  return driver_.convertRadian2Value(id, radian);
}

float DynamixelWorkbench::convertValue2Radian(uint8_t id, int32_t value)
{
  return driver_.convertValue2Radian(id, value);
}

int32_t DynamixelWorkbench::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  return driver_.convertRadian2Value(radian, max_position, min_position, max_radian, min_radian);
}

float DynamixelWorkbench::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  return driver_.convertValue2Radian(value, max_position, min_position, max_radian, min_radian);
}

int32_t DynamixelWorkbench::convertVelocity2Value(uint8_t id, float velocity)
{
  return driver_.convertVelocity2Value(id, velocity);
}

float DynamixelWorkbench::convertValue2Velocity(uint8_t id, int32_t value)
{
  return driver_.convertValue2Velocity(id, value);
}

int16_t DynamixelWorkbench::convertTorque2Value(uint8_t id, float torque)
{
  return driver_.convertTorque2Value(id, torque);
}

float DynamixelWorkbench::convertValue2Torque(uint8_t id, int16_t value)
{
  return driver_.convertValue2Torque(id, value);
}


/*/////////////////////////////////////////////////////////////////////////////
// Private Function
*//////////////////////////////////////////////////////////////////////////////

bool DynamixelWorkbench::torque(uint8_t id, bool onoff)
{
  bool comm_result = false;

  comm_result = driver_.writeRegister(id, "Torque_Enable", onoff);

  return comm_result;
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH"))             ||
        !strncmp(dxl_, "PRO", strlen("PRO")))
    {
      comm_result = driver_.writeRegister(id, "Operating_Mode", X_SERIES_POSITION_CONTROL_MODE);
    }
    else if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2))
    {
      comm_result = driver_.writeRegister(id, "CW_Angle_Limit", 0);
      comm_result = driver_.writeRegister(id, "CCW_Angle_Limit", 1023);
    }
    else
    {
      comm_result = driver_.writeRegister(id, "CW_Angle_Limit", 0);
      comm_result = driver_.writeRegister(id, "CCW_Angle_Limit", 4095);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      comm_result = driver_.writeRegister(id, "CW_Angle_Limit", 0);
      comm_result = driver_.writeRegister(id, "CCW_Angle_Limit", 1023);
    }
    else
      comm_result = driver_.writeRegister(id, "Operating_Mode", X_SERIES_POSITION_CONTROL_MODE);
  }
  millis(10);

  return comm_result;
}

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH"))             ||
        !strncmp(dxl_, "PRO", strlen("PRO")))
    {
      comm_result = driver_.writeRegister(id, "Operating_Mode", X_SERIES_VELOCITY_CONTROL_MODE);
    }
    else
    {
      comm_result = driver_.writeRegister(id, "CW_Angle_Limit", 0);
      comm_result = driver_.writeRegister(id, "CCW_Angle_Limit", 0);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      comm_result = driver_.writeRegister(id, "CW_Angle_Limit", 0);
      comm_result = driver_.writeRegister(id, "CCW_Angle_Limit", 0);
    }
    else
      comm_result = driver_.writeRegister(id, "Operating_Mode", X_SERIES_VELOCITY_CONTROL_MODE);
  } 
  millis(10);

  return comm_result;
}

bool DynamixelWorkbench::setCurrentControlMode(uint8_t id)
{
  bool comm_result = false;

  strcpy(dxl_, driver_.getModelName(id));
  
  if (!strncmp(dxl_, "X", 1)                         ||
      !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) )
  {
    comm_result = driver_.writeRegister(id, "Operating_Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
  }   

  millis(10);

  return comm_result;
}

void DynamixelWorkbench::millis(uint16_t msec)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(msec);
#else
    usleep(1000*msec);
#endif
}
