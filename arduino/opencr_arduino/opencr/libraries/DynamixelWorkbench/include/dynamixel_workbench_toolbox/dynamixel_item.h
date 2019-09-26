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

#ifndef DYNAMIXEL_ITEM_H
#define DYNAMIXEL_ITEM_H

#include <stdint.h>
#include <stddef.h>

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

#define XL430_W250_2 1090 // 2XL

#define XC430_W150 1070
#define XC430_W240 1080

#define XM430_W210 1030
#define XM430_W350 1020

#define XM540_W150 1130
#define XM540_W270 1120

#define XH430_W210 1010
#define XH430_W350 1000
#define XH430_V210 1050
#define XH430_V350 1040

#define XH540_W150 1110
#define XH540_W270 1100
#define XH540_V150 1150
#define XH540_V270 1140

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

#define PRO_M42_10_S260_R_A  43289
#define PRO_M54_40_S250_R_A  46097
#define PRO_M54_60_S250_R_A  46353

#define PRO_H42_20_S300_R_A  51201
#define PRO_H54_100_S500_R_A 53769
#define PRO_H54_200_S500_R_A 54025

#define PRO_PLUS_M42P_010_S260_R  2100
#define PRO_PLUS_M54P_040_S250_R  2110
#define PRO_PLUS_M54P_060_S250_R  2120

#define PRO_PLUS_H42P_020_S300_R  2000
#define PRO_PLUS_H54P_100_S500_R  2010
#define PRO_PLUS_H54P_200_S500_R  2020

#define RH_P12_RN   35073
#define RH_P12_RN_A 35074

#define BYTE  1
#define WORD  2
#define DWORD 4

typedef struct 
{
  const char *item_name;
  uint16_t    address;
  uint8_t	    item_name_length;
  uint16_t    data_length;
} ControlItem;

typedef struct
{
  float rpm;

  int64_t value_of_min_radian_position;
  int64_t value_of_zero_radian_position;
  int64_t value_of_max_radian_position;

  float  min_radian;
  float  max_radian;
} ModelInfo;

// Public Functions
namespace DynamixelItem
{
const ControlItem *getControlTable(uint16_t model_number);
const ModelInfo *getModelInfo(uint16_t model_number);

uint8_t getTheNumberOfControlItem();
}

#endif //DYNAMIXEL_ITEM_H
