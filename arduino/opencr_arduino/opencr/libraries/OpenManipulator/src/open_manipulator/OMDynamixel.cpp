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

/* Authors: Hye-Jong KIM, Darby Lim*/

#include "../../include/open_manipulator/OMKinematics.h"
#if 0

OMDynamixel::OMDynamixel()
{
  dxl_info_.size          = DXL_SIZE;
  dxl_info_.baud_rate     = BAUD_RATE;
  dxl_info_.id_ptr        = NULL;   
  dxl_info_.position_ptr  = NULL;
  dxl_info_.current_ptr   = NULL;
}

OMDynamixel::~OMDynamixel(){};

bool OMDynamixel::init()
{
  dxl_wb_.begin(DEVICE_NAME, dxl_info_.baud_rate);      
    
  if (dxl_wb_.scan(&dxl_id_[0], &dxl_info_.size, 30))
    dxl_info_.id_ptr = &dxl_id_[0];
  else
    return false;

  dxl_wb_.addSyncWrite(GOAL_POSITION);
  if (dxl_wb_.getProtocolVersion() == 2.0)
    dxl_wb_.addSyncRead(PRESENT_POSITION);

  return true;
}

bool OMDynamixel::setMode(uint8_t id, uint8_t mode)
{
  return dxl_wb_.itemWrite(id, OPERATING_MODE, mode);
}

bool OMDynamixel::setPositionControlMode(uint8_t id)
{
  return dxl_wb_.jointMode(id);
}

bool OMDynamixel::setCurrentBasedPositionControlMode(uint8_t id, uint8_t current=10)
{
  return dxl_wb_.currentMode(id, current);
}

bool OMDynamixel::setEnable(uint8_t id)
{
  return dxl_wb_.itemWrite(id, TORQUE_ENABLE, true);
}

bool OMDynamixel::setDisable(uint8_t id)
{
  return dxl_wb_.itemWrite(id, TORQUE_ENABLE, false);
}

bool OMDynamixel::enableAllDynamixel()
{
  for (uint8_t index = 0; index < dxl_info_.size; index++)
    dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, true);

  return true;
}

bool OMDynamixel::disableAllDynamixel()
{
  for (uint8_t index = 0; index < dxl_info_.size; index++)
    dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, false);
  return true;    
}

bool OMDynamixel::setAngle(float *radian)
{
  int32_t set_position[dxl_info_.size] = {0, };
  for (uint8_t index = 0; index < dxl_info_.size; index++)
    set_position[index] = dxl_wb_.convertRadian2Value(dxl_info_.id_ptr[index], radian[index]);

  return dxl_wb_.syncWrite(GOAL_POSITION, set_position);
}

bool OMDynamixel::setAngle(uint8_t id, float radian)
{
  int32_t set_position = dxl_wb_.convertRadian2Value(id, radian);
    
  return dxl_wb_.itemWrite(id, GOAL_POSITION, set_position);   
}

uint8_t* OMDynamixel::getID()
{
  return dxl_info_.id_ptr;
}

uint8_t OMDynamixel::getDynamixelSize()
{
  return dxl_info_.size;
}

uint8_t* OMDynamixel::getDynamixelIds()
{
  return dxl_info_.id_ptr;
}

uint32_t OMDynamixel::getBaudRate()
{
  return dxl_info_.baud_rate;
}

int32_t OMDynamixel::getData(uint8_t id, const char* table_item)
{
  return dxl_wb_.itemRead(id, table_item);
}

float* OMDynamixel::getAngle()
{
  int32_t *get_position_ptr = NULL;
  if (dxl_wb_.getProtocolVersion() == 2.0)
  {
    get_position_ptr = dxl_wb_.syncRead(PRESENT_POSITION);
    
    for (uint8_t index = 0; index < dxl_info_.size; index++)
      radian_value_[index] = dxl_wb_.convertValue2Radian(dxl_info_.id_ptr[index], get_position_ptr[index]);
  }
  else
  {
    for (uint8_t index = 0; index < dxl_info_.size; index++)
      radian_value_[index] = dxl_wb_.convertValue2Radian(dxl_info_.id_ptr[index], dxl_wb_.itemRead(dxl_info_.id_ptr[index], PRESENT_POSITION));
  }

  dxl_info_.position_ptr = &radian_value_[0];
  return dxl_info_.position_ptr;
}

float* OMDynamixel::getCurrent()
{
  int32_t *get_current_ptr = NULL;
  get_current_ptr = dxl_wb_.syncRead(PRESENT_CURRENT);

  for (uint8_t index = 0; index < dxl_info_.size; index++)
    torque_value_[index] = dxl_wb_.convertValue2Torque(dxl_info_.id_ptr[index], get_current_ptr[index]);
  dxl_info_.current_ptr = &torque_value_[0];
  return dxl_info_.current_ptr;
}

void OMDynamixel::addSyncWriteHandler(const char* table_item)
{
  dxl_wb_.addSyncWrite(table_item);
}

void OMDynamixel::addSyncReadHandler(const char* table_item)
{
  dxl_wb_.addSyncRead(table_item);
}
#endif