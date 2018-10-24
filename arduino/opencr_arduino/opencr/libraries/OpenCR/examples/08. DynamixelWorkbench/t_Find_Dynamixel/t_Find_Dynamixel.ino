/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */
/* Updates by Kurt */

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE_NUM 3

DynamixelWorkbench dxl_wb;

void setup()
{
  Serial.begin(57600);
  while (!Serial); // Open a Serial Monitor
  Serial.println("Find Dynamixel Servos test");
  //Serial.print("Size of DynamixelWorkbench: "); Serial.println(sizeof(DynamixelWorkbench), DEC);

}

void loop()
{
  uint8_t scanned_id[100] = {0, };
  uint8_t dxl_cnt = 0;
  uint32_t baud[BAUDRATE_NUM] = {9600, 57600, 1000000};
  uint8_t index = 0;
  uint8_t range = 100;

  while (index < BAUDRATE_NUM)
  {
    Serial.println(String(baud[index]) + " bps");

    dxl_wb.begin(DEVICE_NAME, baud[index]);
    dxl_wb.scan(&scanned_id[0], &dxl_cnt, range);

    for (int i = 0; i < dxl_cnt; i++)
    {
      Serial.print("   id : ");
      Serial.print(scanned_id[i], DEC);
      Serial.print("   Model Name : ");
      Serial.println(dxl_wb.getModelName(scanned_id[i]));
    }

    index++;
  }
  Serial.println("End - Press any key to restart test");
  while (Serial.available() == 0) ;
  while (Serial.available())
  {
    Serial.read();
  }
}
