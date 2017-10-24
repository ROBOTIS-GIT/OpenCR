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

#include "../../include/dynamixel_workbench/dynamixel_tool.h"

DynamixelTool::DynamixelTool(){}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::begin(char* model_name)
{
  strcpy(model_name_, model_name); 
  setControlTable(model_name);
}

bool DynamixelTool::begin(uint16_t model_num)
{
  setControlTable(model_num);
}

void DynamixelTool::setControlTable(char* name)
{  
  if (!strncmp(name, "AX-12A", strlen(name)))
  { 
    setControlTable(AX_12A);
  }
  else if (!strncmp(name, "AX-12W", strlen(name)))
  { 
    setControlTable(AX_12W);
  }
  else if (!strncmp(name, "AX-18A", strlen(name)))
  { 
    setControlTable(AX_18A);
  }

  else if (!strncmp(name, "RX-24F", strlen(name)))
  { 
    setControlTable(RX_24F);
  }
  else if (!strncmp(name, "RX-28", strlen(name)))
  { 
    setControlTable(RX_28);
  }
  else if (!strncmp(name, "RX-64", strlen(name)))
  { 
    setControlTable(RX_64);
  }

  else if (!strncmp(name, "EX-106", strlen(name)))
  { 
    setControlTable(EX_106);
  }

  else if (!strncmp(name, "MX-12W", strlen(name)))
  { 
    setControlTable(MX_12W);
  }
  else if (!strncmp(name, "MX-28", strlen(name)))
  { 
    setControlTable(MX_28);
  }
  else if (!strncmp(name, "MX-64", strlen(name)))
  { 
    setControlTable(MX_64);
  }
  else if (!strncmp(name, "MX-106", strlen(name)))
  { 
    setControlTable(MX_106);
  }

  else if (!strncmp(name, "XL-320", strlen(name)))
  { 
    setControlTable(XL_320);
  }
  else if (!strncmp(name, "XL430-W250", strlen(name)))
  { 
    setControlTable(XL430_W250);
  }

  else if (!strncmp(name, "XM430-W210", strlen(name)))
  { 
    setControlTable(XM430_W210);
  }
  else if (!strncmp(name, "XM430-W350", strlen(name)))
  { 
    setControlTable(XM430_W350);
  }

  else if (!strncmp(name, "XH430-V210", strlen(name)))
  { 
    setControlTable(XH430_V210);
  }
  else if (!strncmp(name, "XH430-V350", strlen(name)))
  { 
    setControlTable(XH430_V350);
  }
  else if (!strncmp(name, "XH430-W210", strlen(name)))
  { 
    setControlTable(XH430_W210);
  }
  else if (!strncmp(name, "XH430-W350", strlen(name)))
  { 
    setControlTable(XH430_W350);
  }
}

void DynamixelTool::setControlTable(uint16_t num)
{
  if (num == AX_12A)
    strcpy(model_name_, "AX-12A");
  else if (num == AX_12W)
    strcpy(model_name_, "AX-12W");
  else if (num == AX_18A)
    strcpy(model_name_, "AX-18A");

  else if (num == RX_24F)
    strcpy(model_name_, "RX-24F");
  else if (num == RX_28)
    strcpy(model_name_, "RX-28");
  else if (num == RX_64)
    strcpy(model_name_, "RX-64");

  else if (num == EX_106)
    strcpy(model_name_, "EX-106");

  else if (num == EX_106)
    strcpy(model_name_, "EX-106");

  else if (num == MX_12W)
    strcpy(model_name_, "MX-12W");
  else if (num == MX_28)
    strcpy(model_name_, "MX-28");
  else if (num == MX_64)
    strcpy(model_name_, "MX-64");
  else if (num == MX_106)
    strcpy(model_name_, "MX-106");

  else if (num == XL_320)
    strcpy(model_name_, "XL-320");
  else if (num == XL430_W250)
    strcpy(model_name_, "XL430-W250");

  else if (num == XM430_W210)
    strcpy(model_name_, "XM430-W210");
  else if (num == XM430_W350)
    strcpy(model_name_, "XM430-W350");

  else if (num == XH430_V210)
    strcpy(model_name_, "XH430-V210");
  else if (num == XH430_V350)
    strcpy(model_name_, "XH430-V350");
  else if (num == XH430_W210)
    strcpy(model_name_, "XH430-W210");
  else if (num == XH430_W350)
    strcpy(model_name_, "XH430-W350");

  item_               = getItem(num);
  control_table_size_ = getSize();
  info_               = getInfo(num);
}

char* DynamixelTool::getModelName()
{
  return model_name_;
}

float DynamixelTool::getVelocityToValueRatio()
{
  return info_->velocity_to_value_ratio;
}

float DynamixelTool::getTorqueToCurrentValueRatio()
{
  return info_->torque_to_current_value_ratio;
}

int32_t DynamixelTool::getValueOfMinRadianPosition()
{
  return info_->value_of_min_radian_position;
}

int32_t DynamixelTool::getValueOfMaxRadianPosition()
{
  return info_->value_of_max_radian_position;
}

int32_t DynamixelTool::getValueOfZeroRadianPosition()
{
  return info_->value_of_0_radian_position;
}

float DynamixelTool::getMinRadian()
{
  return info_->min_radian;
}

float DynamixelTool::getMaxRadian()
{
  return info_->max_radian;
}

uint8_t DynamixelTool::getControlTableSize()
{
  return control_table_size_;
}

void DynamixelTool::setID(uint8_t id)
{
  id_ = id;
}

uint8_t DynamixelTool::getID()
{
  return id_;
}

ControlTableItem* DynamixelTool::getControlItem(char* item_name)
{
  ControlTableItem* cti;

  for (int num = 0; num < control_table_size_; num++)
  {
    if (!strncmp(item_name, item_[num].item_name, strlen(item_[num].item_name)))
    {
      cti = &item_[num];
      return cti;
    }
  }
}
