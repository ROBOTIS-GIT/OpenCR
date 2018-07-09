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

/* Authors: Darby Lim */

#include "../../include/open_manipulator/OMDynamixel.h"

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
OMDynamixel<DXL_SIZE, BAUD_RATE>::OMDynamixel()
{
  dxl_info_.size          = DXL_SIZE;
  dxl_info_.baud_rate     = BAUD_RATE;
  dxl_info_.id_ptr        = NULL;   
  dxl_info_.position_ptr  = NULL;
  dxl_info_.current_ptr   = NULL;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
OMDynamixel<DXL_SIZE, BAUD_RATE>::~OMDynamixel() {}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::init()
{
  dxl_wb_.begin(DEVICE_NAME, dxl_info_.baud_rate);  
  
  uint8_t dxl_id[dxl_info_.size];
  if (dxl_wb_.scan(dxl_id, &dxl_info_.size))
    dxl_info_.id_ptr = dxl_id;
  else
    return false;

  dxl_wb_.addSyncWrite(GOAL_POSITION);

  if (dxl_wb_.getProtocolVersion() == 2.0)
  {
    dxl_wb_.addSyncRead(PRESENT_POSITION);
    dxl_wb_.addSyncRead(PRESENT_CURRENT);
  }

  return true;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setMode(uint8_t id, uint8_t mode)
{
  return dxl_wb_.itemWrite(id, OPERATING_MODE, mode);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setPositionControlMode(uint8_t id)
{
  return dxl_wb_.jointMode(id);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setCurrentBasedPositionControlMode(uint8_t id, uint8_t current)
{
  return dxl_wb_.currentMode(id, current);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setEnable(uint8_t id)
{
  return dxl_wb_.itemWrite(id, TORQUE_ENABLE, true);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setDisable(uint8_t id)
{
  return dxl_wb_.itemWrite(id, TORQUE_ENABLE, false);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::enableAllDynamixel()
{
  for (uint8_t index = 0; index <= dxl_info_.size; index++)
    dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, true);

  return true;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::disableAllDynamixel()
{
  for (uint8_t index = 0; index <= dxl_info_.size; index++)
    dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, false);

  return false;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setAngle(float *data)
{
  int32_t set_position[dxl_info_.size] = {0, };
  for (uint8_t index = 0; index <= dxl_info_.size; index++)
    set_position[index] = dxl_wb_.convertRadian2Value(dxl_info_.id_ptr[index], data[index]);

  return dxl_wb_.syncWrite(GOAL_POSITION, set_position);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
bool OMDynamixel<DXL_SIZE, BAUD_RATE>::setAngle(uint8_t id, float data)
{
  int32_t set_position = dxl_wb_.convertRadian2Value(id, data);
  
  return dxl_wb_.itemWrite(id, GOAL_POSITION, set_position);
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
float* OMDynamixel<DXL_SIZE, BAUD_RATE>::getAngle()
{
  int32_t *get_position_ptr = NULL;
  float radian_value[dxl_info_.size] = {0.0, };

  if (dxl_wb_.getProtocolVersion() == 2.0)
  {
    get_position_ptr = dxl_wb_.syncRead(PRESENT_POSITION);
    
    for (uint8_t index = 0; index <= dxl_info_.size; index++)
      radian_value[index] = dxl_wb_.convertValue2Radian(dxl_info_.id_ptr[index], get_position_ptr[index]);
  }
  else
  {
    for (uint8_t index = 0; index <= dxl_info_.size; index++)
      radian_value[index] = dxl_wb_.convertValue2Radian(dxl_info_.id_ptr[index], dxl_wb_.itemRead(dxl_info_.id_ptr[index], PRESENT_POSITION));
  }

  dxl_info_.position_ptr = radian_value;
  return dxl_info_.position_ptr;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
float* OMDynamixel<DXL_SIZE, BAUD_RATE>::getCurrent()
{
  int32_t *get_current_ptr = NULL;
  float torque_value[dxl_info_.size] = {0.0, };
  get_current_ptr = dxl_wb_.syncRead(PRESENT_POSITION);

  for (uint8_t index = 0; index <= dxl_info_.size; index++)
    torque_value[index] = dxl_wb_.convertValue2Torque(dxl_info_.id_ptr[index], get_current_ptr[index]);

  dxl_info_.current_ptr = torque_value;
  return dxl_info_.current_ptr;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
uint8_t OMDynamixel<DXL_SIZE, BAUD_RATE>::getDynamixelSize()
{
  return dxl_info_.size;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
uint8_t* OMDynamixel<DXL_SIZE, BAUD_RATE>::getDynamixelIds()
{
  return dxl_info_.id_ptr;
}

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
uint32_t OMDynamixel<DXL_SIZE, BAUD_RATE>::getBaudRate()
{
  return dxl_info_.baud_rate;
}