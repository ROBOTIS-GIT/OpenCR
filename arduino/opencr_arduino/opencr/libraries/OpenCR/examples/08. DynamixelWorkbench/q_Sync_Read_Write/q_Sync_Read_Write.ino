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

#define BAUDRATE  57600

DynamixelWorkbench dxl_wb;
uint8_t get_id[5];

int32_t goal_position[2] = {1000, 3000};
int32_t *get_data;

int32_t present_position[2];
int32_t present_velocity[2];

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  dxl_wb.begin(DXL_BUS_SERIAL4, BAUDRATE);
  dxl_wb.scan(get_id);

  dxl_wb.jointMode(get_id[0]);
  dxl_wb.jointMode(get_id[1]);

  dxl_wb.addSyncWrite("Goal Position");
  dxl_wb.addSyncRead("Present Position");
  dxl_wb.addSyncRead("Present Velocity");
}

void loop() 
{  
  dxl_wb.syncWrite("Goal Position", goal_position);

  do
  {
    get_data = dxl_wb.syncRead("Present Velocity");
    present_velocity[0] = get_data[0];
    present_velocity[1] = get_data[1];

    get_data = dxl_wb.syncRead("Present Position");
    present_position[0] = get_data[0];
    present_position[1] = get_data[1];

    log();
  }while(abs(goal_position[0] - present_position[0]) > 20);

  swap();
}

void log()
{
  Serial.print("[ DXL 1:");
  Serial.print(" GoalPos:");  Serial.print(goal_position[0]);
  Serial.print(" PresPos:");  Serial.print(present_position[0]);
  Serial.print(" PresVel:");  Serial.print(present_velocity[0]);
  Serial.print(" ,");
  Serial.print(" DXL 2: ");
  Serial.print(" GoalPos:");  Serial.print(goal_position[1]);
  Serial.print(" PresPos:");  Serial.print(present_position[1]);
  Serial.print(" PresVel:");  Serial.print(present_velocity[1]);
  Serial.println(" ]");
}

void swap()
{
  int32_t tmp = goal_position[0];
  goal_position[0] = goal_position[1];
  goal_position[1] = tmp;
}