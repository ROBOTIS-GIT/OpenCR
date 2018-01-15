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

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "control_table_item.h"

#define AX_12A     12
#define AX_12W     300
#define AX_18A     18

#define RX_10      10
#define RX_24F     24
#define RX_28      28
#define RX_64      64

#define EX_106     107

#define MX_12W     360
#define MX_28      29
#define MX_28_2    30
#define MX_64      310
#define MX_64_2    311
#define MX_106     320
#define MX_106_2   321

#define XL_320     350
#define XL430_W250 1060

#define XM430_W210 1030
#define XM430_W350 1020
#define XM540_W150 1130
#define XM540_W270 1120

#define XH430_V210 1050
#define XH430_V350 1040
#define XH430_W210 1010
#define XH430_W350 1000

#define PRO_L42_10_S300_R  35072
#define PRO_L54_30_S400_R  37928
#define PRO_L54_30_S500_R  37896
#define PRO_L54_50_S290_R  38176
#define PRO_L54_50_S500_R  38152

#define PRO_M42_10_S260_R  43288
#define PRO_M54_40_S250_R  46096
#define PRO_M54_60_S250_R  46352

#define PRO_H42_20_S300_R  51200
#define PRO_H54_100_S500_R 53768
#define PRO_H54_200_S500_R 54024

typedef struct
{
  float velocity_to_value_ratio;
  float torque_to_current_value_ratio;

  int32_t value_of_min_radian_position;
  int32_t value_of_0_radian_position;
  int32_t value_of_max_radian_position;

  float  min_radian;
  float  max_radian;
} ModelInfo;

uint8_t getTheNumberOfControlItem();
ControlTableItem* getConrolTableItem(uint16_t model_number);
ModelInfo* getModelInfo(uint16_t model_number);

#endif //DYNAMIXEL_H
