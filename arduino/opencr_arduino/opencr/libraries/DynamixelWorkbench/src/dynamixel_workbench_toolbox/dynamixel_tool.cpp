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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_tool.h"

//===================================================================
// Define Serial ID to Namd table
//===================================================================
typedef struct {
  uint16_t      model_number;
  const char *  model_name; 
} SERVO_NUM_TO_NAME;

static const SERVO_NUM_TO_NAME servo_num_to_name_table[] = {
    {AX_12A, "AX-12A"},
    {AX_12W, "AX-12W"},
    {AX_18A, "AX-18A"},

    {RX_10, "RX-10"},
    {RX_24F, "RX-24F"},
    {RX_28, "RX-28"},
    {RX_64, "RX-64"},
    {EX_106, "EX-106"},

    {MX_12W, "MX-12W"},
    {MX_28, "MX-28"},
    {MX_28_2, "MX-28-2"},
    {MX_64, "MX-64"},
    {MX_64_2, "MX-64-2"},
    {MX_106, "MX-106"},
    {MX_106_2, "MX-106-2"},

    {XL_320, "XL-320"},
    {XL430_W250, "XL430-W250"},

    {XM430_W210, "XM430-W210"},
    {XM430_W350, "XM430-W350"},
    {XM540_W150, "XM540-W150"},
    {XM540_W270, "XM540-W270"},

    {XH430_V210, "XH430-V210"},
    {XH430_V350, "XH430-V350"},
    {XH430_W210, "XH430-W210"},
    {XH430_W350, "XH430-W350"},

    {PRO_L42_10_S300_R, "PRO-L42-10-S300-R"},
    {PRO_L54_30_S400_R, "PRO-L54-30-S400-R"},
    {PRO_L54_30_S500_R, "PRO-L54-30-S500-R"},
    {PRO_L54_50_S290_R, "PRO-L54-50-S290-R"},
    {PRO_L54_50_S500_R, "PRO-L54-50-S500-R"},

    {PRO_M42_10_S260_R, "PRO-M42-10-S260-R"},
    {PRO_M54_40_S250_R, "PRO-M54-40-S250-R"},
    {PRO_M54_60_S250_R, "PRO-M54-60-S250-R"},

    {PRO_H42_20_S300_R, "PRO-H42-20-S300-R"},
    {PRO_H54_100_S500_R, "PRO-H54-100-S500-R"},
    {PRO_H54_200_S500_R, "PRO-H54-200-S500-R"}
};
#define NUM_SERVO_NUM_TO_NAME  (sizeof(servo_num_to_name_table)/sizeof(servo_num_to_name_table[0]))

DynamixelTool::DynamixelTool() : dxl_info_cnt_(0), the_number_of_item_(0){}

DynamixelTool::~DynamixelTool(){}

void DynamixelTool::addTool(const char* model_name, uint8_t id)
{
  model_name_ = model_name;
  setModelNum(model_name);
  dxl_id_[dxl_info_cnt_] = id;

  setControlTable(model_name);
  dxl_info_cnt_++;
}

void DynamixelTool::addTool(uint16_t model_number, uint8_t id)
{
  setModelName(model_number);
  model_num_ = model_number;
  dxl_id_[dxl_info_cnt_] = id;

  setControlTable(model_number);
  dxl_info_cnt_++;
}

void DynamixelTool::addDXL(const char* model_name, uint8_t id)
{
  model_name_ = model_name;
  setModelNum(model_name);
  dxl_id_[dxl_info_cnt_] = id;

  dxl_info_cnt_++;
}

void DynamixelTool::addDXL(uint16_t model_number, uint8_t id)
{
  setModelName(model_number);
  model_num_ = model_number;
  dxl_id_[dxl_info_cnt_] = id;

  dxl_info_cnt_++;
}

void DynamixelTool::setControlTable(const char *model_name)
{  
  const char* name = model_name;
  uint8_t name_length = strlen(name);

  for (uint8_t index=0; index < NUM_SERVO_NUM_TO_NAME; index++)
  {
    if(strncmp(name, servo_num_to_name_table[index].model_name, name_length) == 0)
    {
      setControlTable(servo_num_to_name_table[index].model_number);
      break;
    }
  }
}

void DynamixelTool::setControlTable(uint16_t model_number)
{
  item_ptr_           = getConrolTableItem(model_number);
  the_number_of_item_ = getTheNumberOfControlItem();
  info_ptr_           = getModelInfo(model_number);

  info_.velocity_to_value_ratio         = info_ptr_->velocity_to_value_ratio;
  info_.torque_to_current_value_ratio   = info_ptr_->torque_to_current_value_ratio;

  info_.value_of_0_radian_position      = info_ptr_->value_of_0_radian_position;
  info_.value_of_min_radian_position    = info_ptr_->value_of_min_radian_position;
  info_.value_of_max_radian_position    = info_ptr_->value_of_max_radian_position;

  info_.min_radian                      = info_ptr_->min_radian;
  info_.max_radian                      = info_ptr_->max_radian;
}

void DynamixelTool::setModelName(uint16_t model_number)
{
  uint16_t num = model_number;

  for (uint8_t index=0; index < NUM_SERVO_NUM_TO_NAME; index++)
  {
    if (num == servo_num_to_name_table[index].model_number)
    {
      model_name_ = servo_num_to_name_table[index].model_name;
      break;
    }
  }
}

void DynamixelTool::setModelNum(const char* model_name)
{
  const char* name = model_name;
  uint8_t name_length = strlen(name);

  for (uint8_t index=0; index < NUM_SERVO_NUM_TO_NAME; index++)
  {
    if(strncmp(name, model_name_, name_length) == 0)
    {
      model_num_ = servo_num_to_name_table[index].model_number;
      break;
    }
  }

}

float DynamixelTool::getVelocityToValueRatio(void)
{
  return info_.velocity_to_value_ratio;
}

float DynamixelTool::getTorqueToCurrentValueRatio(void)
{
  return info_.torque_to_current_value_ratio;
}

int32_t DynamixelTool::getValueOfMinRadianPosition(void)
{
  return info_.value_of_min_radian_position;
}

int32_t DynamixelTool::getValueOfMaxRadianPosition(void)
{
  return info_.value_of_max_radian_position;
}

int32_t DynamixelTool::getValueOfZeroRadianPosition(void)
{
  return info_.value_of_0_radian_position;
}

float DynamixelTool::getMinRadian(void)
{
  return info_.min_radian;
}

float DynamixelTool::getMaxRadian(void)
{
  return info_.max_radian;
}

uint8_t DynamixelTool::getTheNumberOfItem(void)
{
  return the_number_of_item_;
}

const ControlTableItem* DynamixelTool::getControlItem(const char* item_name)
{
  const ControlTableItem* cti = item_ptr_;  
  uint8_t name_length = strlen(item_name);
  for (int num = 0; num < the_number_of_item_; num++)
  {
    if ((name_length == cti->item_name_length) && 
        (memcmp(item_name, cti->item_name, name_length) == 0) )
    {
      return cti;
    }
    cti++;
  }

  if (!strncmp(item_name, "Moving_Speed", strlen("Moving_Speed")))
    getControlItem("Goal_Velocity");
  else if (!strncmp(item_name, "Goal_Velocity", strlen("Goal_Velocity")))
    getControlItem("Moving_Speed");
  else if (!strncmp(item_name, "Present_Velocity", strlen("Present_Velocity")))
    getControlItem("Present_Speed");
  else if (!strncmp(item_name, "Present_Speed", strlen("Present_Speed")))
    getControlItem("Present_Velocity");
  return NULL;
}

const ControlTableItem* DynamixelTool::getControlItemPtr(void)
{
  return item_ptr_;
}

ModelInfo* DynamixelTool::getModelInfoPtr(void)
{
  return info_ptr_;
}
