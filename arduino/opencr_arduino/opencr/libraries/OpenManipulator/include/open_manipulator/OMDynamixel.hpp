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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef OMDYNAMIXEL_HPP_
#define OMDYNAMIXEL_HPP_

#include <DynamixelWorkbench.h>

#define DEVICE_NAME       ""
#define OPERATING_MODE    "Operating_Mode"
#define TORQUE_ENABLE     "Torque_Enable"
#define GOAL_POSITION     "Goal_Position"
#define PRESENT_POSITION  "Present_Position"
#define PRESENT_CURRENT   "Present_Current"

typedef struct
{
  uint32_t baud_rate;
  uint8_t* id_ptr = NULL;
  uint8_t size;
  float* position_ptr = NULL;
  float* current_ptr = NULL;
}DXL_INFO;

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
class OMDynamixel
{
 private:
  DynamixelWorkbench dxl_wb_;
  DXL_INFO dxl_info_;

  uint8_t dxl_id_[DXL_SIZE];
  float radian_value_[DXL_SIZE];
  float torque_value_[DXL_SIZE];

 public:
  OMDynamixel()
  {
    dxl_info_.size          = DXL_SIZE;
    dxl_info_.baud_rate     = BAUD_RATE;
    dxl_info_.id_ptr        = NULL;   
    dxl_info_.position_ptr  = NULL;
    dxl_info_.current_ptr   = NULL;
  }

  ~OMDynamixel(){};

  bool init()
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

  bool setMode(uint8_t id, uint8_t mode)
  {
    return dxl_wb_.itemWrite(id, OPERATING_MODE, mode);
  }

  bool setPositionControlMode(uint8_t id)
  {
    return dxl_wb_.jointMode(id);
  }

  bool setCurrentBasedPositionControlMode(uint8_t id, uint8_t current=10)
  {
    return dxl_wb_.currentMode(id, current);
  }

  bool setEnable(uint8_t id)
  {
    return dxl_wb_.itemWrite(id, TORQUE_ENABLE, true);
  }

  bool setDisable(uint8_t id)
  {
    return dxl_wb_.itemWrite(id, TORQUE_ENABLE, false);
  }

  bool enableAllDynamixel()
  {
    for (uint8_t index = 0; index < dxl_info_.size; index++)
      dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, true);

    return true;
  }

  bool disableAllDynamixel()
  {
    for (uint8_t index = 0; index < dxl_info_.size; index++)
      dxl_wb_.itemWrite(dxl_info_.id_ptr[index], TORQUE_ENABLE, false);

    return false;    
  }

  bool setAngle(float *data)
  {
    int32_t set_position[dxl_info_.size] = {0, };
    for (uint8_t index = 0; index < dxl_info_.size; index++)
      set_position[index] = dxl_wb_.convertRadian2Value(dxl_info_.id_ptr[index], data[index]);

    return dxl_wb_.syncWrite(GOAL_POSITION, set_position);
  }

  bool setAngle(uint8_t id, float data)
  {
    int32_t set_position = dxl_wb_.convertRadian2Value(id, data);
    
    return dxl_wb_.itemWrite(id, GOAL_POSITION, set_position);   
  }

  uint8_t* getID()
  {
    return dxl_info_.id_ptr;
  }

  uint8_t getDynamixelSize()
  {
    return dxl_info_.size;
  }

  uint8_t* getDynamixelIds()
  {
    return dxl_info_.id_ptr;
  }

  uint32_t getBaudRate()
  {
    return dxl_info_.baud_rate;
  }

  int32_t getData(uint8_t id, const char* table_item)
  {
    return dxl_wb_.itemRead(id, table_item);
  }

  float* getAngle()
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

  float* getCurrent()
  {
    int32_t *get_current_ptr = NULL;
    get_current_ptr = dxl_wb_.syncRead(PRESENT_CURRENT);

    for (uint8_t index = 0; index < dxl_info_.size; index++)
      torque_value_[index] = dxl_wb_.convertValue2Torque(dxl_info_.id_ptr[index], get_current_ptr[index]);

    dxl_info_.current_ptr = &torque_value_[0];
    return dxl_info_.current_ptr;
  }

  void addSyncWriteHandler(const char* table_item)
  {
    dxl_wb_.addSyncWrite(table_item);
  }

  void addSyncReadHandler(const char* table_item)
  {
    dxl_wb_.addSyncRead(table_item);
  }
};

#endif // OMDYNAMIXEL_HPP_
