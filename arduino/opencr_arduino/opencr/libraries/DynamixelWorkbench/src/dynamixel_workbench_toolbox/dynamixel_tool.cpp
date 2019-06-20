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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_tool.h"

//===================================================================
// Define Serial ID to Namd table
//===================================================================
typedef struct 
{
  uint16_t      number;
  const char*   name; 
} DynamixelModel;

static const DynamixelModel dynamixel_model_table[] = {
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

    {XL430_W250_2, "XL430-W250-2"}, // 2XL

    {XC430_W150, "XC430-W150"},
    {XC430_W240, "XC430-W240"},

    {XM430_W210, "XM430-W210"},
    {XM430_W350, "XM430-W350"},
    
    {XM540_W150, "XM540-W150"},
    {XM540_W270, "XM540-W270"},

    {XH430_V210, "XH430-V210"},
    {XH430_V350, "XH430-V350"},
    {XH430_W210, "XH430-W210"},
    {XH430_W350, "XH430-W350"},

    {XH540_W150, "XH540_W150"},
    {XH540_W270, "XH540_W270"},
    {XH540_V150, "XH540_V150"},
    {XH540_V270, "XH540_V270"},

    {PRO_L42_10_S300_R, "PRO-L42-10-S300-R"},
    {PRO_L54_30_S400_R, "PRO-L54-30-S400-R"},
    {PRO_L54_30_S500_R, "PRO-L54-30-S500-R"},
    {PRO_L54_50_S290_R, "PRO-L54-50-S290-R"},
    {PRO_L54_50_S500_R, "PRO-L54-50-S500-R"},

    {PRO_M42_10_S260_R, "PRO-M42-10-S260-R"},
    {PRO_M54_40_S250_R, "PRO-M54-40-S250-R"},
    {PRO_M54_60_S250_R, "PRO-M54-60-S250-R"},

    {PRO_H42_20_S300_R,  "PRO-H42-20-S300-R"},
    {PRO_H54_100_S500_R, "PRO-H54-100-S500-R"},
    {PRO_H54_200_S500_R, "PRO-H54-200-S500-R"},

    {PRO_M42_10_S260_R_A, "PRO-M42-10-S260-R-A"},
    {PRO_M54_40_S250_R_A, "PRO-M54-40-S250-R-A"},
    {PRO_M54_60_S250_R_A, "PRO-M54-60-S250-R-A"},

    {PRO_H42_20_S300_R_A,  "PRO-H42-20-S300-R-A"},
    {PRO_H54_100_S500_R_A, "PRO-H54-100-S500-R-A"},
    {PRO_H54_200_S500_R_A, "PRO-H54-200-S500-R-A"},

    {PRO_PLUS_M42P_010_S260_R, "PRO-PLUS-M42P-010-S260-R"},
    {PRO_PLUS_M54P_040_S250_R, "PRO-PLUS-M54P-040-S250-R"},
    {PRO_PLUS_M54P_060_S250_R, "PRO-PLUS-M54P-060-S250-R"},

    {PRO_PLUS_H42P_020_S300_R, "PRO-PLUS-H42P-020-S300-R"},
    {PRO_PLUS_H54P_100_S500_R, "PRO-PLUS-H54P-100-S500-R"},
    {PRO_PLUS_H54P_200_S500_R, "PRO-PLUS-H54P-200-S500-R"},

    {RH_P12_RN, "RH-P12-RN"},

    {RH_P12_RN_A, "RH-P12-RN-A"}
};
#define COUNT_DYNAMIXEL_MODEL  (sizeof(dynamixel_model_table)/sizeof(dynamixel_model_table[0]))

DynamixelTool::DynamixelTool() : dxl_cnt_(0), the_number_of_control_item_(0){}

DynamixelTool::~DynamixelTool(){}

void DynamixelTool::initTool(void)
{
  for (uint8_t i = 0; i < DYNAMIXEL_BUFFER; i++)
    dxl_id_[i] = 0;

  dxl_cnt_ = 0;
}

bool DynamixelTool::addTool(const char *model_name, uint8_t id, const char **log)
{
  bool result = false;
  initTool();

  model_name_ = model_name;
  result = setModelNumber(model_name, log);
  if (result == false) return false;
  dxl_id_[dxl_cnt_++] = id;

  result = setControlTable(model_name, log);
  if (result == false) return false;

  return true;
}

bool DynamixelTool::addTool(uint16_t model_number, uint8_t id, const char **log)
{
  bool result = false;
  initTool();

  result = setModelName(model_number, log);
  if (result == false) return false;
  model_number_ = model_number;
  dxl_id_[dxl_cnt_++] = id;

  result = setControlTable(model_number, log);
  if (result == false) return false;

  return result;
}

void DynamixelTool::addDXL(uint8_t id)
{
  dxl_id_[dxl_cnt_++] = id;
}

bool DynamixelTool::setControlTable(const char *model_name, const char **log)
{  
  const char* name = model_name;
  uint8_t name_length = strlen(name);

  for (uint8_t index=0; index < COUNT_DYNAMIXEL_MODEL; index++)
  {
    if(strncmp(name, dynamixel_model_table[index].name, name_length) == 0)
    {
      return setControlTable(dynamixel_model_table[index].number, log);
    }
  }

  if (log != NULL)
    *log = "[DynamixelTool] Failed to set control table due to mismatch model name and model number";
  return false;
}

bool DynamixelTool::setControlTable(uint16_t model_number, const char **log)
{
  control_table_              = DynamixelItem::getControlTable(model_number);
  the_number_of_control_item_ = DynamixelItem::getTheNumberOfControlItem();
  model_info_                 = DynamixelItem::getModelInfo(model_number);

  if (control_table_ == NULL || model_info_ == NULL)
  {
    if (log != NULL)
      *log = "[DynamixelTool] Failed to get control table or model info";
    return false;
  }

  return true;
}

bool DynamixelTool::setModelName(uint16_t model_number, const char **log)
{
  uint16_t num = model_number;

  for (uint8_t index=0; index < COUNT_DYNAMIXEL_MODEL; index++)
  {
    if (num == dynamixel_model_table[index].number)
    {
      model_name_ = dynamixel_model_table[index].name;
      return true;
    }
  }

  if (log != NULL)
    *log = "[DynamixelTool] Failed to find model name";
  return false;
}

bool DynamixelTool::setModelNumber(const char *model_name, const char **log)
{
  const char* name = model_name;
  uint8_t name_length = strlen(name);

  for (uint8_t index=0; index < COUNT_DYNAMIXEL_MODEL; index++)
  {
    if(strncmp(name, model_name_, name_length) == 0)
    {
      model_number_ = dynamixel_model_table[index].number;
      return true;
    }
  }

  if (log != NULL)
    *log = "[DynamixelTool] Failed to find model number";
  return false;
}

const char* DynamixelTool::getModelName(void)
{
  return model_name_;
}

uint16_t DynamixelTool::getModelNumber(void)
{
  return model_number_;
}

const uint8_t* DynamixelTool::getID(void)
{
  const uint8_t* id_table_ = dxl_id_;

  return id_table_;
}

uint8_t DynamixelTool::getDynamixelCount(void)
{
  return dxl_cnt_;
}

uint8_t DynamixelTool::getDynamixelBuffer(void)
{
  return DYNAMIXEL_BUFFER;
}

float DynamixelTool::getRPM(void)
{
  return model_info_->rpm;
}

int64_t DynamixelTool::getValueOfMinRadianPosition(void)
{
  return model_info_->value_of_min_radian_position;
}

int64_t DynamixelTool::getValueOfMaxRadianPosition(void)
{
  return model_info_->value_of_max_radian_position;
}

int64_t DynamixelTool::getValueOfZeroRadianPosition(void)
{
  return model_info_->value_of_zero_radian_position;
}

float DynamixelTool::getMinRadian(void)
{
  return model_info_->min_radian;
}

float DynamixelTool::getMaxRadian(void)
{
  return model_info_->max_radian;
}

uint8_t DynamixelTool::getTheNumberOfControlItem(void)
{
  return the_number_of_control_item_;
}

const ControlItem *DynamixelTool::getControlItem(const char *item_name, const char **log)
{
  const ControlItem* control_item = control_table_;  
  uint8_t name_length = strlen(item_name);

  for (uint8_t num = 0; num < the_number_of_control_item_; num++)
  {
    if ((name_length == control_item->item_name_length) && 
        (memcmp(item_name, control_item->item_name, name_length) == 0))
    {
      return control_item;
    }
    control_item++;
  }

  if (log != NULL)
    *log = "[DynamixelTool] Can't find Item";
  return NULL;
}

const ControlItem *DynamixelTool::getControlTable(void)
{
  return control_table_;
}

const ModelInfo *DynamixelTool::getModelInfo(void)
{
  return model_info_;
}

