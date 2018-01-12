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

#define STRING_BUF_NUM 64
String cmd[STRING_BUF_NUM];

DynamixelWorkbench dxl_wb;
uint8_t get_id[16];
uint8_t dxl_cnt = 0;

void setup() 
{
  Serial.begin(57600);
  while(!Serial); // Open a Serial Monitor  

  Serial.println("-------------------------------------");
  Serial.println("Set portHandler Before scan or ping");
  Serial.println("-------------------------------------");
  Serial.println("begin  (BAUD)");
  Serial.println("scan   (RANGE)");
  Serial.println("ping   (ID)");
  Serial.println("id     (ID) (NEW_ID)");
  Serial.println("baud   (ID) (NEW_BAUD)");
  Serial.println("torque (ID) (VALUE)");
  Serial.println("joint  (ID) (GOAL_POSITION)");
  Serial.println("wheel  (ID) (GOAL_VELOCITY)");
  Serial.println("write  (ID) (ADDRESS_NAME) (VALUE)");
  Serial.println("read   (ID) (ADDRESS_NAME)");
  Serial.println("reboot (ID) ");
  Serial.println("reset  (ID) ");
  Serial.println("-------------------------------------");
}

void loop() 
{
  if (Serial.available())
  {
    String read_string = Serial.readStringUntil('\n');
    Serial.println("[CMD] : " + String(read_string));

    read_string.trim();

    split(read_string, ' ', cmd);

    if (cmd[0] == "begin")
    {
      uint32_t baud = cmd[1].toInt();
      if (dxl_wb.begin(DXL_BUS_SERIAL4, baud))
        Serial.println("Succeed to begin");
      else
        Serial.println("Failed to begin");
    }
    else if (cmd[0] == "scan")
    { 
      uint8_t range = cmd[1].toInt();
      dxl_wb.scan(get_id, &dxl_cnt, range);

      if (dxl_cnt == 0)
        Serial.println("Can't find Dynamixel");
      else
      {
        Serial.println("Find " + String(dxl_cnt) + " Dynamixels");
        for (int cnt = 0; cnt < dxl_cnt; cnt++)
          Serial.println("ID : " + String(get_id[cnt]));
      }
    }
    else if (cmd[0] == "ping")
    {
      uint8_t id = cmd[1].toInt();
      uint16_t model_number = 0;

      if (dxl_wb.ping(id, &model_number))
        Serial.println("id : " + String(id) + " model_number : " + String(model_number));
      else
        Serial.println("Failed to ping");
    }
    else if (cmd[0] == "id")
    {
      uint8_t id     = cmd[1].toInt();
      uint8_t new_id = cmd[2].toInt();

      if (dxl_wb.setID(id, new_id))
        Serial.println("Succeed to change ID");
      else
        Serial.println("Failed");

    }
    else if (cmd[0] == "baud")
    {
      uint8_t  id       = cmd[1].toInt();
      uint32_t  new_baud  = cmd[2].toInt();

      if (dxl_wb.setBaud(id, new_baud))
        Serial.println("Succeed to change BaudRate");
      else
        Serial.println("Failed");
    }
    else if (cmd[0] == "torque")
    {
      uint8_t id       = cmd[1].toInt();
      uint8_t onoff    = cmd[2].toInt();

      if (dxl_wb.itemWrite(id, "Torque_Enable", onoff))
        Serial.println("Succeed to torque command!!");
      else
        Serial.println("Failed");
    }
    else if (cmd[0] == "joint")
    {
      uint8_t id    = cmd[1].toInt();
      uint16_t goal = cmd[2].toInt();

      dxl_wb.jointMode(id);
      if (dxl_wb.goalPosition(id, goal))
       Serial.println("Succeed to joint command!!");
      else
        Serial.println("Failed");
    }
    else if (cmd[0] == "wheel")
    {
      uint8_t id    = cmd[1].toInt();
      int32_t goal  = cmd[2].toInt();

      dxl_wb.wheelMode(id);
      if (dxl_wb.goalSpeed(id, goal))
        Serial.println("Succeed to wheel command!!");
      else
        Serial.println("Failed");
    }
    else if (cmd[0] == "write")
    {
      uint8_t id = cmd[1].toInt();      
      int32_t value = cmd[3].toInt(); 

      if (dxl_wb.itemWrite(id, cmd[2].c_str(), value))
      {        
        Serial.println("Succeed to write command!!");
      }
      else
        Serial.println("Failed");
    }
    else if (cmd[0] == "read")
    {
      uint8_t id = cmd[1].toInt();
            
      int32_t value = dxl_wb.itemRead(id, cmd[2].c_str());

      Serial.println("read data : " + String(value));
    }
    else if (cmd[0] == "reboot")
    {
      uint8_t id = cmd[1].toInt();

      if (dxl_wb.reboot(id))
        Serial.println("Succeed to reboot");
      else
        Serial.println("Failed to reboot");
    }
    else if (cmd[0] == "reset")
    {
      uint8_t id = cmd[1].toInt();

      if (dxl_wb.reset(id))
        Serial.println("Succeed to reset");
      else
        Serial.println("Failed to reset");
    }
    else 
    {
      Serial.println("Wrong command");
    }
  }
}

void split(String data, char separator, String* temp)
{
	int cnt = 0;
	int get_index = 0;

  String copy = data;
  
	while(true)
	{
		get_index = copy.indexOf(separator);

		if(-1 != get_index)
		{
			temp[cnt] = copy.substring(0, get_index);

			copy = copy.substring(get_index + 1);
		}
		else
		{
      temp[cnt] = copy.substring(0, copy.length());
			break;
		}
		++cnt;
	}
}