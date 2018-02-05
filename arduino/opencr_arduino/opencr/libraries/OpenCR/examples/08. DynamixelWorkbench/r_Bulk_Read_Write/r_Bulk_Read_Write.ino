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

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    

#define BAUDRATE  57600
#define DXL_ID_1  1
#define DXL_ID_2  2

DynamixelWorkbench dxl_wb;
uint8_t dxl_id[2] = {DXL_ID_1, DXL_ID_2};
uint8_t dxl_cnt = 2;

int32_t goal_position[2] = {1000, 3000};
int32_t led_status[2] = {true, false};

int32_t present_position;
int32_t present_led;

void setup() 
{
  Serial.begin(57600);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  for (int cnt = 0; cnt < dxl_cnt; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt]);
    dxl_wb.jointMode(dxl_id[cnt]);
  }

  dxl_wb.initBulkWrite();
  dxl_wb.initBulkRead();

  dxl_wb.addBulkReadParam(dxl_id[0], "Present_Position");
  dxl_wb.addBulkReadParam(dxl_id[1], "LED");
}

void loop() 
{  
  static uint8_t index = 0;

  dxl_wb.addBulkWriteParam(dxl_id[0], "Goal_Position", goal_position[index]);
  dxl_wb.addBulkWriteParam(dxl_id[1], "LED", led_status[index]);

  dxl_wb.bulkWrite();

  do
  {
    dxl_wb.setBulkRead();
    
    present_position = dxl_wb.bulkRead(dxl_id[0], "Present_Position");
    present_led      = dxl_wb.bulkRead(dxl_id[1], "LED");

    log(index);
  }while(abs(goal_position[index] - present_position) > 20);

  if (index == 0)
    index = 1;
  else
    index = 0;
}

void log(int index)
{
  Serial.print("[ DXL 1  GoalPos : " + String(goal_position[index]) +
               " PresPos : "         + String(present_position)     +
               " , DXL 2 LED : "     + String(present_led));
  Serial.println(" ]");
}