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

#define DXL_BUS_SERIAL1 "1"            //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 "2"            //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 "3"            //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define DXL_BUS_SERIAL4 "/dev/ttyUSB0" //Dynamixel on Serial3(USART3)  <-OpenCR

#define BAUDRATE  1000000

DynamixelWorkbench dxl_wb;
uint8_t get_id[5];

int32_t goal_position[2] = {1000, 3000};
int32_t led_status[2] = {true, false};

int32_t present_position;
int32_t present_led;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  dxl_wb.begin(DXL_BUS_SERIAL4, BAUDRATE);
  dxl_wb.scan(get_id);

  dxl_wb.jointMode(get_id[0]);
  dxl_wb.jointMode(get_id[1]);

  dxl_wb.initBulkWrite();
  dxl_wb.initBulkRead();

  dxl_wb.addBulkReadParam(get_id[0], "Present Position");
  dxl_wb.addBulkReadParam(get_id[1], "LED");
}

void loop() 
{  
  static uint8_t index = 0;

  dxl_wb.addBulkWriteParam(get_id[0], "Goal Position", goal_position[index]);
  dxl_wb.addBulkWriteParam(get_id[1], "LED", led_status[index]);

  dxl_wb.bulkWrite();

  do
  {
    dxl_wb.setBulkRead();
    
    present_position = dxl_wb.bulkRead(get_id[0], "Present Position");
    present_led      = dxl_wb.bulkRead(get_id[1], "LED");

    log(index);
  }while(abs(goal_position[index] - present_position) > 20);

  if (index == 0)
    index = 1;
  else
    index = 0;
}

void log(int index)
{
  Serial.print("[ DXL 1:");
  Serial.print(" GoalPos:" + String(goal_position[index]));
  Serial.print(" PresPos:" + String(present_position));
  Serial.print(" , DXL 2: ");
  Serial.print(" LED:" + String(present_led));
  Serial.println(" ]");
}