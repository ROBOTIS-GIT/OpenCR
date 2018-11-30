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
#define DXL_ID  1
#define NEW_DXL_ID  2

DynamixelWorkbench dxl_wb;

void setup() 
{
  Serial.begin(57600);
  while(!Serial); // Wait for Opening Serial Monitor

  const char *log;
  bool result = false;

  uint8_t dxl_id = DXL_ID;
  uint8_t dxl_new_id = NEW_DXL_ID;
  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.print(dxl_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }

  result = dxl_wb.changeID(dxl_id, dxl_new_id, &log);
  if (result == false)
  {
    Serial.println(log);
    return;
  }
  else
  {
    Serial.println(log);
  }

  uint8_t scanned_id[16];
  uint8_t dxl_cnt = 0;
  uint8_t range = 100;

  result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to scan");
  }
  else
  {
    Serial.print("Find ");
    Serial.print(dxl_cnt);
    Serial.println(" Dynamixels");

    for (int cnt = 0; cnt < dxl_cnt; cnt++)
    {
      Serial.print("id : ");
      Serial.print(scanned_id[cnt]);
      Serial.print(" model name : ");
      Serial.println(dxl_wb.getModelName(scanned_id[cnt]));
    }
  }
}

void loop() 
{

}