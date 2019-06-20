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

/* Authors: Taehun Lim (Darby), Ryan Shim */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"

static const uint8_t WHEEL_MODE = 1;
static const uint8_t JOINT_MODE = 2;

static const uint8_t CURRENT_CONTROL_MODE                  = 0;
static const uint8_t VELOCITY_CONTROL_MODE                 = 1;
static const uint8_t POSITION_CONTROL_MODE                 = 3;
static const uint8_t EXTENDED_POSITION_CONTROL_MODE        = 4;
static const uint8_t CURRENT_BASED_POSITION_CONTROL_MODE   = 5;
static const uint8_t PWM_CONTROL_MODE                      = 16;
static const uint8_t TORQUE_CONTROL_MODE                   = 100;
static const uint8_t MULTI_TURN_MODE                       = 101;

static const char* model_name = NULL;
static const ModelInfo* model_info = NULL;

DynamixelWorkbench::DynamixelWorkbench(){}

DynamixelWorkbench::~DynamixelWorkbench(){}

bool DynamixelWorkbench::torque(uint8_t id, int32_t onoff, const char **log)
{
  bool result = false;

  result = itemWrite(id, "Torque_Enable", (int32_t)onoff, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to change torque status!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to change torque status!";
  return result;
}

bool DynamixelWorkbench::torqueOn(uint8_t id, const char **log)
{
  bool result = false;

  result = torque(id, 1, log);

  return result;
}

bool DynamixelWorkbench::torqueOff(uint8_t id, const char **log)
{
  bool result = false;

  result = torque(id, 0, log);

  return result;
}

bool DynamixelWorkbench::changeID(uint8_t id, uint8_t new_id, const char **log)
{
  bool result = false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = writeRegister(id, "ID", new_id, log);
  if (result == false) 
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to change ID!";
    return false;
  }
  // millis(1000);

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to change ID!";
  return result;
}

bool DynamixelWorkbench::changeBaudrate(uint8_t id, uint32_t new_baudrate, const char **log)
{
  bool result = false;

  result = torqueOff(id, log);
  if (result == false) return false;

  if (getProtocolVersion() == 1.0f)
  {
    switch (new_baudrate)
    {
      case 9600:
        result = writeRegister(id, "Baud_Rate", 207, log);
       break;

      case 19200:
        result = writeRegister(id, "Baud_Rate", 103, log);
       break;

      case 57600:
        result = writeRegister(id, "Baud_Rate", 34, log);
       break;

      case 115200:
        result = writeRegister(id, "Baud_Rate", 16, log);
       break;

      case 200000:
        result = writeRegister(id, "Baud_Rate", 9, log);    
       break;

      case 250000:
        result = writeRegister(id, "Baud_Rate", 7, log);       
       break;
       
      case 400000:
        result = writeRegister(id, "Baud_Rate", 4, log);       
       break;

      case 500000:
        result = writeRegister(id, "Baud_Rate", 3, log);       
       break;

      case 1000000:
        result = writeRegister(id, "Baud_Rate", 1, log);       
       break;

      case 2250000:
        result = writeRegister(id, "Baud_Rate", 250, log);       
       break;

      case 2500000:
        result = writeRegister(id, "Baud_Rate", 251, log);       
       break;

      case 3000000:
        result = writeRegister(id, "Baud_Rate", 252, log);       
       break;
       
      default:
        result = writeRegister(id, "Baud_Rate", 34, log);
       break;
    }
  }
  else if (getProtocolVersion() == 2.0f)
  {    
    switch (new_baudrate)
    {
      case 9600:
        result = writeRegister(id, "Baud_Rate", 0, log);       
       break;

      case 57600:
        result = writeRegister(id, "Baud_Rate", 1, log);       
       break;

      case 115200:
        result = writeRegister(id, "Baud_Rate", 2, log);       
       break;

      case 1000000:
        result = writeRegister(id, "Baud_Rate", 3, log);       
       break;

      case 2000000:
        result = writeRegister(id, "Baud_Rate", 4, log);       
       break;

      case 3000000:
        result = writeRegister(id, "Baud_Rate", 5, log);       
       break;
       
      case 4000000:
        result = writeRegister(id, "Baud_Rate", 6, log);       
       break;

      case 4500000:
        result = writeRegister(id, "Baud_Rate", 7, log);       
       break;

      case 10500000:
        result = writeRegister(id, "Baud_Rate", 8, log);       
       break;
       
      default:
        result = writeRegister(id, "Baud_Rate", 1, log);       
       break;
    }
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to change Baud Rate!";
    return result; 
  } 

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to change Baud Rate!";
  return result;
}

bool DynamixelWorkbench::changeProtocolVersion(uint8_t id, uint8_t version, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH")))
  {    
    result = writeRegister(id, "Protocol_Version", version, log);
    if (result == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set protocol version!";
      return false;
    }
  }

  result = setPacketHandler((float)version, log);
  
  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set protocol version!";
  return result;
}

bool DynamixelWorkbench::itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  return writeRegister(id, item_name, data, log);
}

bool DynamixelWorkbench::itemRead(uint8_t id, const char *item_name, int32_t *data, const char **log)
{
  return readRegister(id, item_name, data, log);
}

bool DynamixelWorkbench::led(uint8_t id, int32_t onoff, const char **log)
{
  bool result = false;

  result = writeRegister(id, "LED", onoff, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to change led status!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to change led status!";
  return result;
}

bool DynamixelWorkbench::ledOn(uint8_t id, const char **log)
{
  bool result = false;

  result = led(id, 1, log);

  return result;
}

bool DynamixelWorkbench::ledOff(uint8_t id, const char **log)
{
  bool result = false;

  result = led(id, 0, log);

  return result;
}

bool DynamixelWorkbench::setNormalDirection(uint8_t id, const char **log)
{
  bool result = false;
  int32_t data = 0;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH")))
  {    
    result = readRegister(id, "Drive_Mode", &data, log);
    
    data = data & 0b00000100;
    result = writeRegister(id, "Drive_Mode", data, log);
    if (result == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set normal direction!";
      return false;
    }
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set normal direction!";
  return result;
}

bool DynamixelWorkbench::setReverseDirection(uint8_t id, const char **log)
{
  bool result = false;
  int32_t data = 0;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH")))
  {
    result = readRegister(id, "Drive_Mode", &data, log);
    
    data = data | 0b00000001;
    result = writeRegister(id, "Drive_Mode", data, log);
    if (result == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set reverse direction!";
      return false;
    }
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set reverse direction!";
  return result;
}

bool DynamixelWorkbench::setVelocityBasedProfile(uint8_t id, const char **log)
{
  bool result = false;
  int32_t data = 0;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH")))
  {
    result = readRegister(id, "Drive_Mode", &data, log);
    
    data = data & 0b00000001;
    result = writeRegister(id, "Drive_Mode", data, log);
    if (result == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set velocity based profile!";
      return false;
    }
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set velocity based profile!";
  return result;
}

bool DynamixelWorkbench::setTimeBasedProfile(uint8_t id, const char **log)
{
  bool result = false;
  int32_t data = 0;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH")))
  {
    result = readRegister(id, "Drive_Mode", &data, log);
    
    data = data | 0b00000100;
    result = writeRegister(id, "Drive_Mode", data, log);
    if (result == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set time based profile!";
      return false;
    }
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set time based profile!";
  return result;
}

bool DynamixelWorkbench::setSecondaryID(uint8_t id, uint8_t secondary_id, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
      !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XL430", strlen("XL430"))       ||
      !strncmp(model_name, "XC430", strlen("XC430"))       ||
      !strncmp(model_name, "XH", strlen("XH"))             ||
      !strncmp(model_name, "RH", strlen("RH")))
  {
    result = torqueOff(id, log);
    if (result == false) return false;

    result = writeRegister(id, "Secondary_ID", secondary_id, log);
    if (result == false) 
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to set secondary ID!";
      return false;
    }
  }

  // millis(1000);

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set secondary ID!";
  return result;
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, POSITION_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Position Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Position Control Mode!";
  return result;
}

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, VELOCITY_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Velocity Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Velocity Control Mode!";
  return result;
}

bool DynamixelWorkbench::setCurrentControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, CURRENT_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Current Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Current Control Mode!";
  return result;
}

bool DynamixelWorkbench::setTorqueControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, TORQUE_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Torque Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Torque Control Mode!";
  return result;
}

bool DynamixelWorkbench::setExtendedPositionControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, EXTENDED_POSITION_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Extended Position Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Extended Position Control Mode!";
  return result;
}

bool DynamixelWorkbench::setMultiTurnControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, MULTI_TURN_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Multi-Turn Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Multi-Turn Control Mode!";
  return result;
}

bool DynamixelWorkbench::setCurrentBasedPositionControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, CURRENT_BASED_POSITION_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Current Based Position Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Current Based Position Control Mode!";
  return result;
}

bool DynamixelWorkbench::setPWMControlMode(uint8_t id, const char **log)
{
  bool result = false;

  result = setOperatingMode(id, PWM_CONTROL_MODE, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set PWM Control Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set PWM Control Mode!";
  return result;
}

bool DynamixelWorkbench::setOperatingMode(uint8_t id, uint8_t index, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (getProtocolVersion() == 1.0)
  {
    if (index == POSITION_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XL430", strlen("XL430"))       ||
          !strncmp(model_name, "XC430", strlen("XC430"))       ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "PRO", strlen("PRO"))           ||
          !strncmp(model_name, "RH", strlen("RH")))
      {
        result = writeRegister(id, "Operating_Mode", POSITION_CONTROL_MODE, log);
      }
      else if (!strncmp(model_name, "AX", 2) || !strncmp(model_name, "RX", 2))
      {
        result = writeRegister(id, "CW_Angle_Limit", 0, log);
        result = writeRegister(id, "CCW_Angle_Limit", 1023, log);
      }
      else
      {
        result = writeRegister(id, "CW_Angle_Limit", 0, log);
        result = writeRegister(id, "CCW_Angle_Limit", 4095, log);
      }
    }
    else if (index == VELOCITY_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XL430", strlen("XL430"))       ||
          !strncmp(model_name, "XC430", strlen("XC430"))       ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "PRO", strlen("PRO")))
      {
        result = writeRegister(id, "Operating_Mode", VELOCITY_CONTROL_MODE);
      }
      else
      {
        result = writeRegister(id, "CW_Angle_Limit",  0, log);
        result = writeRegister(id, "CCW_Angle_Limit", 0, log);
      }
    }
    else if (index == CURRENT_CONTROL_MODE)
    {
      if (!strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "RH", strlen("RH")))
      {
        result = writeRegister(id, "Operating_Mode", CURRENT_CONTROL_MODE, log);
      }  
    }
    else if (index == TORQUE_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-64", strlen("MX-64"))   ||
          !strncmp(model_name, "MX-106", strlen("MX-106")) )
      {
        result = writeRegister(id, "Torque_Control_Mode_Enable", 1, log);
      }
    }
    else if (index == MULTI_TURN_MODE)
    {
      if (!strncmp(model_name, "MX-64", strlen("MX-64"))   ||
          !strncmp(model_name, "MX-106", strlen("MX-106")) )
      {
        result = writeRegister(id, "CW_Angle_Limit",  4095, log);
        result = writeRegister(id, "CCW_Angle_Limit", 4095, log);
      }
    }
    else if (index == CURRENT_BASED_POSITION_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "RH", strlen("RH")))
      {
        result = writeRegister(id, "Operating_Mode", CURRENT_BASED_POSITION_CONTROL_MODE, log);
      }
    }
    else if (index == PWM_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH")))
      {
        result = writeRegister(id, "Operating_Mode", PWM_CONTROL_MODE, log);
      }
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (index == POSITION_CONTROL_MODE)
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320")))
        result = writeRegister(id, "Control_Mode", JOINT_MODE, log);
      else
        result = writeRegister(id, "Operating_Mode", POSITION_CONTROL_MODE, log);
    }
    else if (index == VELOCITY_CONTROL_MODE)
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320")))
        result = writeRegister(id, "Control_Mode", WHEEL_MODE, log);
      else
        result = writeRegister(id, "Operating_Mode", VELOCITY_CONTROL_MODE, log);
    }
    else if (index == CURRENT_CONTROL_MODE)
    {
      if (!strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "RH", strlen("RH")))
      {
        result = writeRegister(id, "Operating_Mode", CURRENT_CONTROL_MODE, log);
      }  
    }
    else if (index == TORQUE_CONTROL_MODE)
    {
      if (!strncmp(model_name, "PRO", strlen("PRO"))   ||
          strncmp(model_name, "PRO-L42", strlen("PRO-L42")) )
      {
        result = writeRegister(id, "Operating_Mode", 0, log);
      }
    }
    else if (index == EXTENDED_POSITION_CONTROL_MODE)
    {
      if (!strncmp(model_name, "PRO", strlen("PRO"))   ||
          strncmp(model_name, "PRO-L42", strlen("PRO-L42")) )
      {
        result = writeRegister(id, "Operating_Mode", EXTENDED_POSITION_CONTROL_MODE, log);
      }
    }
    else if (index == CURRENT_BASED_POSITION_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "RH", strlen("RH")))
      {
        result = writeRegister(id, "Operating_Mode", CURRENT_BASED_POSITION_CONTROL_MODE, log);
      }
    }
    else if (index == PWM_CONTROL_MODE)
    {
      if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
          !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
          !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
          !strncmp(model_name, "XM", strlen("XM"))             ||
          !strncmp(model_name, "XH", strlen("XH"))             ||
          !strncmp(model_name, "XL430", strlen("XL430"))       ||
          !strncmp(model_name, "XC430", strlen("XC430"))       ||
          !strncmp(model_name, "PRO", strlen("PRO")))
      {
        result = writeRegister(id, "Operating_Mode", PWM_CONTROL_MODE, log);
      }
    }
  }

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Operating Mode!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Operating Mode!";
  return result;
}


bool DynamixelWorkbench::jointMode(uint8_t id, int32_t velocity, int32_t acceleration, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = setPositionControlMode(id, log);
  if (result == false) return false;

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(model_name, "XL430", strlen("XL430"))       ||
        !strncmp(model_name, "XC430", strlen("XC430"))       ||
        !strncmp(model_name, "XM", strlen("XM"))             ||
        !strncmp(model_name, "XH", strlen("XH")))
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
      result = writeRegister(id, "Profile_Velocity", velocity, log);
    }
    else if (!strncmp(model_name, "MX-28", strlen("MX-28"))   ||
             !strncmp(model_name, "MX-64", strlen("MX-64"))   ||
             !strncmp(model_name, "MX-106", strlen("MX-106")))
    {
      result = writeRegister(id, "Moving_Speed", velocity, log);
      result = writeRegister(id, "Goal_Acceleration", acceleration, log);
    }
    else
    {
      result = writeRegister(id, "Moving_Speed", velocity, log);
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(model_name, "XL-320", strlen("XL-320")))
    {
      result = writeRegister(id, "Moving_Speed", velocity, log);
    }
    else if (!strncmp(model_name, "PRO-M42-10-S260-R-A",  strlen("PRO-M42-10-S260-R-A"))  ||
             !strncmp(model_name, "PRO-M54-40-S250-R-A",  strlen("PRO-M54-40-S250-R-A"))  ||
             !strncmp(model_name, "PRO-M54-60-S250-R-A",  strlen("PRO-M54-60-S250-R-A"))  ||
             !strncmp(model_name, "PRO-H42-20-S300-R-A",  strlen("PRO-H42-20-S300-R-A"))  ||
             !strncmp(model_name, "PRO-H54-100-S500-R-A", strlen("PRO-H54-100-S500-R-A")) ||
             !strncmp(model_name, "PRO-H54-200-S500-R-A", strlen("PRO-H54-200-S500-R-A")))
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
      result = writeRegister(id, "Profile_Velocity", velocity, log);
    }
    else if (!strncmp(model_name, "PRO-L", strlen("PRO-L")) ||
             !strncmp(model_name, "PRO-M", strlen("PRO-M")) ||
             !strncmp(model_name, "PRO-H", strlen("PRO-H")))
    {
      result = writeRegister(id, "Goal_Velocity", velocity, log);
      result = writeRegister(id, "Goal_Acceleration", acceleration, log);
    }
    else
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
      result = writeRegister(id, "Profile_Velocity", velocity, log);
    }
  }

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Joint Mode!";
    return false;
  }

  result = torqueOn(id, log);
  if (result == false) return false;

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Joint Mode!";
  return result;
}

bool DynamixelWorkbench::wheelMode(uint8_t id, int32_t acceleration, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = setVelocityControlMode(id, log);
  if (result == false) return false;

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(model_name, "XL430", strlen("XL430"))       ||
        !strncmp(model_name, "XC430", strlen("XC430"))       ||
        !strncmp(model_name, "XM", strlen("XM"))             ||
        !strncmp(model_name, "XH", strlen("XH")))
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
    }
    else if (!strncmp(model_name, "MX-28", strlen("MX-28"))   ||
             !strncmp(model_name, "MX-64", strlen("MX-64"))   ||
             !strncmp(model_name, "MX-106", strlen("MX-106")))
    {
      result = writeRegister(id, "Goal_Acceleration", acceleration, log);
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(model_name, "PRO-PLUS", strlen("PRO-PLUS")))
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
    }
    else if (!strncmp(model_name, "PRO", strlen("PRO")))  
    {
      result = writeRegister(id, "Goal_Acceleration", acceleration, log);
    }
    else
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
    }
  }

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Wheel Mode!";
    return false;
  }

  result = torqueOn(id, log);
  if (result == false) return false;

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Wheel Mode!";
  return result;
}

bool DynamixelWorkbench::currentBasedPositionMode(uint8_t id, int32_t current, const char **log)
{
  bool result = false;

  model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = setCurrentBasedPositionControlMode(id, log);
  if (result == false) return false;

  if (!strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
      !strncmp(model_name, "XM", strlen("XM"))             ||
      !strncmp(model_name, "XH", strlen("XH"))             ||
      !strncmp(model_name, "RH", strlen("RH")))
  {   
    result = writeRegister(id, "Goal_Current", current, log);
  }

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set Current Based Position Mode!";
    return false;
  }

  result = torqueOn(id, log);
  if (result == false) return false;

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set Current Based Position Wheel Mode!";
  return result;
}

//keep compatibility with older codes
bool DynamixelWorkbench::goalPosition(uint8_t id, int value, const char **log)
{
  goalPosition(id, (int32_t)value, log);
}

bool DynamixelWorkbench::goalPosition(uint8_t id, int32_t value, const char **log)
{
  bool result = false;
  
  result = itemWrite(id, "Goal_Position", value, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal position!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal position!";
  return result;
}

//keep compatibility with older codes
bool DynamixelWorkbench::goalSpeed(uint8_t id, int value, const char **log)
{
  bool result = false;
  result = goalVelocity(id, (int32_t)value, log);
  return result;
}

//keep compatibility with older codes
bool DynamixelWorkbench::goalVelocity(uint8_t id, int value, const char **log)
{
  goalVelocity(id, (int32_t)value, log);
}

bool DynamixelWorkbench::goalVelocity(uint8_t id, int32_t value, const char **log)
{
  bool result[2] = {false, false};

  if (getProtocolVersion() == 2.0f)
  {
    result[0] = writeRegister(id, "Goal_Velocity", value, log);
    if (result[0] == false)
    {
      if (value < 0)
      {
        value = (-1) * value;
        value |= 1024;
      }
      result[1] = writeRegister(id, "Moving_Speed", value, log);
      if (result[1] == false)
      {
        if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal velocity!";
        return false;
      }
      else
      {
        if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal velocity!";
        return true;
      }
    }
    else
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal velocity!";
      return true;
    }
  }
  else
  {
    result[0] = writeRegister(id, "Goal_Velocity", value, log);
    if (result[0] == false)
    {
      if (value < 0)
      {
        value = (-1) * value;
        value |= 1024;
      }
      result[1] = writeRegister(id, "Moving_Speed", value, log);
      if (result[1] == false)
      {
        if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal velocity!";
        return false;
      }
      else
      {
        if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal velocity!";
        return true;
      }
    }
    else
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal velocity!";
      return true;
    }
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal velocity!";
  return false;
}

bool DynamixelWorkbench::goalPosition(uint8_t id, float radian, const char **log)
{
  bool result = 0;
  uint32_t value = 0;

  value = convertRadian2Value(id, radian);

  result = goalPosition(id, (int32_t)value, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal position!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal position!";
  return true;
}

bool DynamixelWorkbench::goalVelocity(uint8_t id, float velocity, const char **log)
{
  bool result = 0;
  int32_t value = 0;

  value = convertVelocity2Value(id, velocity);

  result = goalVelocity(id, value, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal velocity!";
    return result;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal velocity!";
  return result;
}

bool DynamixelWorkbench::getPresentPositionData(uint8_t id, int32_t* data, const char **log)
{
  bool result = 0;
  int32_t get_data = 0;

  result = readRegister(id, "Present_Position", &get_data, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to get present position data!";
    return result;
  }

  *data = get_data;

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to get present position data!";
  return result;
}

bool DynamixelWorkbench::getRadian(uint8_t id, float* radian, const char **log)
{
  bool result = 0;
  int32_t get_data = 0;

  result = getPresentPositionData(id, &get_data, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to get radian!";
    return result;
  }

  *radian = convertValue2Radian(id, get_data);

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to get radian!";
  return result;
}

bool DynamixelWorkbench::getVelocity(uint8_t id, float* velocity, const char **log)
{
  bool result = 0;
  int32_t get_data = 0;

  result = getPresentVelocityData(id, &get_data, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to get velocity!";
    return result;
  }

  *velocity = convertValue2Velocity(id, get_data);

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to get velocity!";
  return result;
}

bool DynamixelWorkbench::getPresentVelocityData(uint8_t id, int32_t* data, const char **log)
{
  bool result[2] = {false, false};
  int32_t get_data = 0;

  result[0] = readRegister(id, "Goal_Velocity", &get_data, log);
  if (result[0] == false)
  {
    result[1] = readRegister(id, "Moving_Speed", &get_data, log);
    if (result[1] == false)
    {
      if (log != NULL) *log = "[DynamixelWorkbench] Failed to get goal velocity!";
      return false;
    }
    else
    {
      *data = get_data;
      if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to get goal velocity!";
      return true;
    }
  }
  else
  {
    *data = get_data;
    if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to get goal velocity!";
    return true;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Failed to get goal velocity!";
  return false;
}

int32_t DynamixelWorkbench::convertRadian2Value(uint8_t id, float radian)
{
  int32_t position = 0;

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (radian > 0)
  {
    position = (radian * (model_info->value_of_max_radian_position - model_info->value_of_zero_radian_position) / model_info->max_radian) + model_info->value_of_zero_radian_position;
  }
  else if (radian < 0)
  {
    position = (radian * (model_info->value_of_min_radian_position - model_info->value_of_zero_radian_position) / model_info->min_radian) + model_info->value_of_zero_radian_position;
  }
  else
  {
    position = model_info->value_of_zero_radian_position;
  }

  return position;
}

float DynamixelWorkbench::convertValue2Radian(uint8_t id, int32_t value)
{
  float radian = 0.0;

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (value > model_info->value_of_zero_radian_position)
  {
    radian = (float)(value - model_info->value_of_zero_radian_position) * model_info->max_radian / (float)(model_info->value_of_max_radian_position - model_info->value_of_zero_radian_position);
  }
  else if (value < model_info->value_of_zero_radian_position)
  {
    radian = (float)(value - model_info->value_of_zero_radian_position) * model_info->min_radian / (float)(model_info->value_of_min_radian_position - model_info->value_of_zero_radian_position);
  }

  return radian;
}

int32_t DynamixelWorkbench::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  int32_t value = 0;
  int32_t zero_position = (max_position + min_position)/2;

  if (radian > 0)
  {
    value = (radian * (max_position - zero_position) / max_radian) + zero_position;
  }
  else if (radian < 0)
  {
    value = (radian * (min_position - zero_position) / min_radian) + zero_position;
  }
  else
  {
    value = zero_position;
  }

  return value;
}

float DynamixelWorkbench::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  float radian = 0.0;
  int32_t zero_position = (max_position + min_position)/2;

  if (value > zero_position)
  {
    radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
  }
  else if (value < zero_position)
  {
    radian = (float)(value - zero_position) * min_radian / (float)(min_position - zero_position);
  }

  return radian;
}

int32_t DynamixelWorkbench::convertVelocity2Value(uint8_t id, float velocity)
{
  int32_t value = 0;
  const float RPM2RADPERSEC = 0.104719755f;

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (getProtocolVersion() == 1.0f)
  {
    if (strncmp(getModelName(id), "AX", strlen("AX")) == 0 ||
        strncmp(getModelName(id), "RX", strlen("RX")) == 0 ||
        strncmp(getModelName(id), "EX", strlen("EX")) == 0 ||
        strncmp(getModelName(id), "MX", strlen("MX")) == 0)
    {
      if (velocity == 0.0f) value = 0;
      else if (velocity < 0.0f) value = (velocity / (model_info->rpm * RPM2RADPERSEC));
      else if (velocity > 0.0f) value = (velocity / (model_info->rpm * RPM2RADPERSEC)) + 1023;

      return value;
    }
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if (strcmp(getModelName(id), "XL-320") == 0)
    {
      if (velocity == 0.0f) value = 0;
      else if (velocity < 0.0f) value = (velocity / (model_info->rpm * RPM2RADPERSEC));
      else if (velocity > 0.0f) value = (velocity / (model_info->rpm * RPM2RADPERSEC)) + 1023;

      return value;
    }
    else
    {
      value = velocity / (model_info->rpm * RPM2RADPERSEC);
    }

    return value;
  }

  return 0;
}

float DynamixelWorkbench::convertValue2Velocity(uint8_t id, int32_t value)
{
  float velocity = 0;
  const float RPM2RADPERSEC = 0.104719755f;

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (getProtocolVersion() == 1.0f)
  {
    if (strncmp(getModelName(id), "AX", strlen("AX")) == 0 ||
        strncmp(getModelName(id), "RX", strlen("RX")) == 0 ||
        strncmp(getModelName(id), "EX", strlen("EX")) == 0 ||
        strncmp(getModelName(id), "MX", strlen("MX")) == 0)
    {
      if (value == 1023 || value == 0) velocity = 0.0f;
      else if (value > 0 && value < 1023) velocity = value * model_info->rpm * RPM2RADPERSEC;
      else if (value > 1023 && value < 2048) velocity = (value - 1023) * model_info->rpm * RPM2RADPERSEC  * (-1.0f);

      return velocity;
    }
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if (strcmp(getModelName(id), "XL-320") == 0)
    {
      if (value == 1023 || value == 0) velocity = 0.0f;
      else if (value > 0 && value < 1023) velocity = value * model_info->rpm * RPM2RADPERSEC;
      else if (value > 1023 && value < 2048) velocity = (value - 1023) * model_info->rpm * RPM2RADPERSEC * (-1.0f);
    }
    else
    {
      velocity = value * (model_info->rpm * RPM2RADPERSEC);
    }

    return velocity;
  }

  return 0.0f;
}

int16_t DynamixelWorkbench::convertCurrent2Value(uint8_t id, float current)
{
  float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (getProtocolVersion() == 1.0f)
  {
      return (current / CURRENT_UNIT);
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if (strncmp(getModelName(id), "PRO-L", strlen("PRO-L")) == 0 ||
        strncmp(getModelName(id), "PRO-M", strlen("PRO-M")) == 0 ||
        strncmp(getModelName(id), "PRO-H", strlen("PRO-H")) == 0)
    {
      CURRENT_UNIT = 16.11328f;
      return (current / CURRENT_UNIT);
    }
    else if (strncmp(getModelName(id), "PRO-PLUS", strlen("PRO-PLUS")) == 0)
    {
      CURRENT_UNIT = 1.0f;
      return (current / CURRENT_UNIT);
    }
    else
    {
      return (current / CURRENT_UNIT);
    }
  }

  return (current / CURRENT_UNIT);
}

int16_t DynamixelWorkbench::convertCurrent2Value(float current)
{
  int16_t value = 0;
  const float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  value = current / CURRENT_UNIT;

  return value;
}

float DynamixelWorkbench::convertValue2Current(uint8_t id, int16_t value)
{
  float current = 0;
  float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;

  if (getProtocolVersion() == 1.0f)
  {
    current = (int16_t)value * CURRENT_UNIT;
    return current;
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if (strncmp(getModelName(id), "PRO-L", strlen("PRO-L")) == 0 ||
        strncmp(getModelName(id), "PRO-M", strlen("PRO-M")) == 0 ||
        strncmp(getModelName(id), "PRO-H", strlen("PRO-H")) == 0)
    {
      CURRENT_UNIT = 16.11328f;
      current = (int16_t)value * CURRENT_UNIT;
      return current;
    }
    else if (strncmp(getModelName(id), "PRO-PLUS", strlen("PRO-PLUS")) == 0)
    {
      CURRENT_UNIT = 1.0f;
      current = (int16_t)value * CURRENT_UNIT;
      return current;
    }
    else
    {
      current = (int16_t)value * CURRENT_UNIT;
      return current;
    }
  }

  current = (int16_t)value * CURRENT_UNIT;

  return current;
}

float DynamixelWorkbench::convertValue2Current(int16_t value)
{
  float current = 0;
  const float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  current = (int16_t)value * CURRENT_UNIT;

  return current;
}

float DynamixelWorkbench::convertValue2Load(int16_t value)
{
  float load = 0;
  const float LOAD_UNIT = 0.1f; //Unit : %, Ref : http://emanual.robotis.com/docs/en/dxl/mx/mx-28/#present-load

  if (value == 1023 || value == 0) load = 0.0f;
  else if (value > 0 && value < 1023) load = value * LOAD_UNIT;
  else if (value > 1023 && value < 2048) load = (value - 1023) * LOAD_UNIT * (-1.0f);

  return load;
}
