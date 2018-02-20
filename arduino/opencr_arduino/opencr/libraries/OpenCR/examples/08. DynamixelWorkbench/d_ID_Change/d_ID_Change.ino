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

#define BAUDRATE    57600
#define DXL_ID      1
#define NEW_DXL_ID  2

DynamixelWorkbench dxl_wb;

void setup() 
{
  Serial.begin(57600);
  while(!Serial); // If this line is activated, you need to open Serial Terminal.

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb.ping(DXL_ID);

  if (dxl_wb.setID(DXL_ID, NEW_DXL_ID))
  {
    dxl_wb.ping(NEW_DXL_ID);

    Serial.println("Succeed to change ID");
    Serial.print("Dynamixel NEW ID : " + String(NEW_DXL_ID));

    dxl_wb.jointMode(NEW_DXL_ID);
  }
  else
  {
    Serial.println("Failed to change ID");
  }
}

void loop() 
{
  dxl_wb.goalPosition(NEW_DXL_ID, 0);
  
  delay(2000);

  dxl_wb.goalPosition(NEW_DXL_ID, 2000);

  delay(2000);
}