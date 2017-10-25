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

#include "../../include/dynamixel_workbench/dynamixel_workbench.h"

DynamixelWorkbench::DynamixelWorkbench()
{

}

DynamixelWorkbench::~DynamixelWorkbench()
{

}

bool DynamixelWorkbench::begin(char* device_name, uint32_t baud_rate)
{
  bool error = false;

  error = driver_.begin(device_name, baud_rate);

  return error;
}

uint8_t  DynamixelWorkbench::scan(uint8_t *get_id, float protocol_version)
{
  uint8_t id_cnt = 0;

  id_cnt = driver_.scan(get_id, 16, protocol_version);

  return id_cnt;
}

uint16_t DynamixelWorkbench::ping(uint8_t id, float protocol_version)
{
  uint16_t model_num = 0;

  model_num = driver_.ping(id, protocol_version);

  return model_num;
}

bool DynamixelWorkbench::reboot(uint8_t id)
{
  bool error = false;

  error = driver_.reboot(id);

  return error;
}

bool DynamixelWorkbench::reset(uint8_t id)
{
  bool error = false;

  error = driver_.reset(id);

  return error;
}

bool DynamixelWorkbench::setID(uint8_t id, uint8_t new_id)
{
  bool check = false;

  torque(id, FALSE);

  check = driver_.writeRegister(id, "ID", new_id);

#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(1000);
#else
  sleep(1);
#endif

  return check;
}

bool DynamixelWorkbench::setBaud(uint8_t id, uint32_t new_baud)
{
  bool check = false;

  torque(id, FALSE);

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (new_baud == 9600)
      check = driver_.writeRegister(id, "Baud Rate", 207);
    else if (new_baud == 57600)
      check = driver_.writeRegister(id, "Baud Rate", 34);
    else if (new_baud == 115200)
      check = driver_.writeRegister(id, "Baud Rate", 16);
    else if (new_baud == 1000000)
      check = driver_.writeRegister(id, "Baud Rate", 1);
    else if (new_baud == 2000000)
      check = driver_.writeRegister(id, "Baud Rate", 9);
    else
      check = driver_.writeRegister(id, "Baud Rate", 1);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (new_baud == 9600)
      check = driver_.writeRegister(id, "Baud Rate", 0);
    else if (new_baud == 57600)
      check = driver_.writeRegister(id, "Baud Rate", 1);
    else if (new_baud == 115200)
      check = driver_.writeRegister(id, "Baud Rate", 2);
    else if (new_baud == 1000000)
      check = driver_.writeRegister(id, "Baud Rate", 3);
    else if (new_baud == 2000000)
      check = driver_.writeRegister(id, "Baud Rate", 4);
    else
      check = driver_.writeRegister(id, "Baud Rate", 3);
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(1000);
#else
  sleep(1);
#endif

  return check;
}

bool DynamixelWorkbench::setPacketHandler(float protocol_version)
{
  driver_.setPacketHandler(protocol_version);
}

char* DynamixelWorkbench::getModelName(uint8_t id)
{
  driver_.getModelName(id);
}

bool DynamixelWorkbench::ledOn(uint8_t id, int32_t data)
{
  driver_.writeRegister(id, "LED", data);
}

bool DynamixelWorkbench::ledOff(uint8_t id)
{
  driver_.writeRegister(id, "LED", 0);
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  strcpy(dxl_, driver_.getModelName(id));

  torque(id, FALSE);

  setPositionControlMode(id);

  torque(id, TRUE);

  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "Moving Speed", vel);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (!strncmp(dxl_, "XL-320", 6) || !strncmp(dxl_, "PRO", 3))
    {
      driver_.writeRegister(id, "Moving Speed", vel);
    }
    else
    {
      driver_.writeRegister(id, "Profile Acceleration", acc);
      driver_.writeRegister(id, "Profile Velocity", vel);
    }
  }
}

bool DynamixelWorkbench::wheelMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  strcpy(dxl_, driver_.getModelName(id));

  torque(id, FALSE);

  setVelocityControlMode(id);

  torque(id, TRUE);

  if (driver_.getProtocolVersion() == 2.0 && (strncmp(dxl_, "PRO", 3) != 0))
  {   
    driver_.writeRegister(id, "Profile Acceleration", acc);
    driver_.writeRegister(id, "Profile Velocity", vel);
  }
}

bool DynamixelWorkbench::currentMode(uint8_t id, uint8_t cur)
{
  strcpy(dxl_, driver_.getModelName(id));
  
  torque(id, FALSE);

  setCurrentControlMode(id);

  torque(id, TRUE);

  if (!strncmp(dxl_, "X", 1))
  {   
    driver_.writeRegister(id, "Goal Current", cur);
  }
}

bool DynamixelWorkbench::goalPosition(uint8_t id, uint16_t goal)
{
  bool check = false;
  
  check = driver_.writeRegister(id, "Goal Position", goal);

  return check;
}

bool DynamixelWorkbench::goalSpeed(uint8_t id, int32_t goal)
{
  bool check = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (goal < 0)
    {
      goal = (-1) * goal;
      goal |= 1024;
    }
    check = driver_.writeRegister(id, "Moving Speed", goal);
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
      check = driver_.writeRegister(id, "Moving Speed", goal);
    }
    else
      check = driver_.writeRegister(id, "Goal Velocity", goal);  
  }

  return check;
}

bool DynamixelWorkbench::regWrite(uint8_t id, char* item_name, int32_t value)
{
  driver_.writeRegister(id, item_name, value);
}

bool DynamixelWorkbench::syncWrite(char *item_name, int32_t* value)
{
  return driver_.syncWrite(item_name, value);
}

bool DynamixelWorkbench::bulkWrite()
{
  return driver_.bulkWrite();
}

int32_t DynamixelWorkbench::regRead(uint8_t id, char* item_name)
{
  static int32_t value = 0;

  if (driver_.readRegister(id, item_name, &value))
    return value;
}

int32_t* DynamixelWorkbench::syncRead(char *item_name)
{
  static int32_t data[16];
  if (driver_.syncRead(item_name, data))
    return data;
}

int32_t DynamixelWorkbench::bulkRead(uint8_t id, char* item_name)
{
  static int32_t data;
  if (driver_.bulkRead(id, item_name, &data))
    return data;
}

bool DynamixelWorkbench::addSyncWrite(char* item_name)
{
  driver_.addSyncWrite(item_name);

  return true;
}

bool DynamixelWorkbench::addSyncRead(char* item_name)
{
  driver_.addSyncRead(item_name);

  return true;
}

bool DynamixelWorkbench::initBulkWrite()
{
  driver_.initBulkWrite();

  return true;
}

bool DynamixelWorkbench::initBulkRead()
{
  driver_.initBulkRead();

  return true;
}

bool DynamixelWorkbench::addBulkWriteParam(uint8_t id, char *item_name, int32_t data)
{
  return driver_.addBulkWriteParam(id, item_name, data);
}

bool DynamixelWorkbench::addBulkReadParam(uint8_t id, char *item_name)
{
  return driver_.addBulkReadParam(id, item_name);
}

bool DynamixelWorkbench::setBulkRead()
{
  return driver_.sendBulkReadPacket();
}


/*/////////////////////////////////////////////////////////////////////////////
// Private Function
*//////////////////////////////////////////////////////////////////////////////

bool DynamixelWorkbench::torque(uint8_t id, bool onoff)
{
  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "Torque ON/OFF", onoff);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      driver_.writeRegister(id, "Torque ON/OFF", onoff);
    }
    else
    {
      driver_.writeRegister(id, "Torque Enable", onoff);
    }
  }
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id)
{
  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 1023);
    }
    else
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 4095);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 1023);
      driver_.writeRegister(id, "Control Mode", XL320_POSITION_CONTROL_MODE);
    }
    else
      driver_.writeRegister(id, "Operating Mode", X_SERIES_POSITION_CONTROL_MODE);
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  sleep(0.01);
#endif
}

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id)
{
  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "CW Angle Limit", 0);
    driver_.writeRegister(id, "CCW Angle Limit", 0);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 0);
      driver_.writeRegister(id, "Control Mode", XL320_VELOCITY_CONTROL_MODE);
    }
    else
      driver_.writeRegister(id, "Operating Mode", X_SERIES_VELOCITY_CONTROL_MODE);
  } 
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  sleep(0.01);
#endif  
}

bool DynamixelWorkbench::setCurrentControlMode(uint8_t id)
{
  strcpy(dxl_, driver_.getModelName(id));
  
  if (!strncmp(dxl_, "X", 1))
  {
    driver_.writeRegister(id, "Operating Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
  }   
  else
  {
    Serial.println("Position control based current control is only support in X series");
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  sleep(0.01);
#endif
}