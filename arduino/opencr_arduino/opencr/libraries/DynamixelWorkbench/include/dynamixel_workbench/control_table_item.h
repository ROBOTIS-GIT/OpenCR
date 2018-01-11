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

#ifndef CONTROL_TABLE_ITEM_H
#define CONTROL_TABLE_ITEM_H

#include <stdint.h>

typedef struct 
{
  uint16_t    address;
  const char* item_name;
  uint8_t     data_length;
} ControlTableItem;

#endif //CONTROL_TABLE_ITEM_H
