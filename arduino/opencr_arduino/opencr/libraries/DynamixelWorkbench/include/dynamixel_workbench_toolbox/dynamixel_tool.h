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

#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#include <string.h>

#include "dynamixel_item.h"

#if defined(__OPENCR__) || defined(__OPENCM904__)
  #define ITEM_ARRAY_SIZE 14
#else
  #define ITEM_ARRAY_SIZE 60
#endif

typedef struct
{
  char model_name[20];
  uint16_t model_num;
  uint8_t id;
} DXLInfo;

class DynamixelTool
{
 public:
  DXLInfo dxl_info_[16];
  uint8_t dxl_info_cnt_;

 private:
  ControlTableItem* item_ptr_;
  ModelInfo* info_ptr_;

#if defined(__OPENCR__) || defined(__OPENCM904__)
  ControlTableItem item_[ITEM_ARRAY_SIZE];
#else
  ControlTableItem item_[ITEM_ARRAY_SIZE];
#endif

  ModelInfo info_;
  uint8_t the_number_of_item_;

 public:
  DynamixelTool();
  ~DynamixelTool();

  void addTool(const char* model_name, uint8_t id);
  void addTool(uint16_t model_number, uint8_t id);

  void addDXL(uint16_t model_number, uint8_t id);
  void addDXL(const char* model_name, uint8_t id);

  float getVelocityToValueRatio(void);
  float getTorqueToCurrentValueRatio(void);

  int32_t getValueOfMinRadianPosition(void);
  int32_t getValueOfMaxRadianPosition(void);
  int32_t getValueOfZeroRadianPosition(void);

  float getMinRadian(void);
  float getMaxRadian(void);

  uint8_t getTheNumberOfItem(void);
  ControlTableItem* getControlItem(const char *item_name);
  ControlTableItem* getControlItemPtr(void);
  ModelInfo* getModelInfoPtr(void);

 private:
  void setControlTable(const char* model_name);
  void setControlTable(uint16_t model_number);

  void setModelName(uint16_t model_number);
  void setModelNum(const char* model_name);
};
#endif //DYNAMIXEL_TOOL_H
