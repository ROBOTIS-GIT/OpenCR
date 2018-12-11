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

#define STRING_BUF_NUM 64
String cmd[STRING_BUF_NUM];

DynamixelWorkbench dxl_wb;
uint8_t get_id[16];
uint8_t scan_cnt = 0;
uint8_t ping_cnt = 0;

bool isAvailableID(uint8_t id);
void split(String data, char separator, String* temp);
void printInst();

void setup() 
{
  Serial.begin(57600);
  while(!Serial); // Open a Serial Monitor  

  printInst();
}

void loop() 
{
  const char *log = NULL;
  bool result = false;

  if (Serial.available())
  {
    String read_string = Serial.readStringUntil('\n');
    Serial.println("[CMD] : " + String(read_string));

    read_string.trim();

    split(read_string, ' ', cmd);

    if (cmd[0] == "help")
    {
      printInst();
    }
    else if (cmd[0] == "begin")
    {
      if (cmd[1] == '\0')
        cmd[1] = String("57600");

      uint32_t baud = cmd[1].toInt();
      result = dxl_wb.init(DEVICE_NAME, baud);
      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to init");
      }
      else
      {
        Serial.print("Succeed to init : ");
        Serial.println(baud);  
      }
    }
    else if (cmd[0] == "end")
    {        
      return;
    }
    else if (cmd[0] == "scan")
    { 
      if (cmd[1] == '\0')
        cmd[1] = String("253");

      uint8_t range = cmd[1].toInt();
      result = dxl_wb.scan(get_id, &scan_cnt, range);
      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to scan");
      }
      else
      {
        Serial.print("Find ");
        Serial.print(scan_cnt);
        Serial.println(" Dynamixels");

        for (int cnt = 0; cnt < scan_cnt; cnt++)
        {
          Serial.print("id : ");
          Serial.print(get_id[cnt]);
          Serial.print(" model name : ");
          Serial.println(dxl_wb.getModelName(get_id[cnt]));
        }
      }  
    }
    else if (cmd[0] == "ping")
    {
      if (cmd[1] == '\0')
        cmd[1] = String("1");

      get_id[ping_cnt] = cmd[1].toInt();
      uint16_t model_number = 0;

      result = dxl_wb.ping(get_id[ping_cnt], &model_number, &log);
      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to ping");
      }
      else
      {
        ping_cnt++;

        Serial.println("Succeed to ping");
        Serial.print("id : ");
        Serial.print(get_id[ping_cnt]);
        Serial.print(" model_number : ");
        Serial.println(model_number);
      }
    }
    else if (isAvailableID(cmd[1].toInt()))
    {
      if (cmd[0] == "control_table")
      {
        uint8_t id = cmd[1].toInt();

        const ControlItem *control_item =  dxl_wb.getControlTable(id);
        uint8_t the_number_of_control_item = dxl_wb.getTheNumberOfControlItem(id);

        uint16_t last_register_addr = control_item[the_number_of_control_item-1].address;
        uint16_t last_register_addr_length = control_item[the_number_of_control_item-1].data_length;

        uint32_t getAllRegisteredData[last_register_addr+last_register_addr_length];

        if (control_item != NULL)
        {
          result = dxl_wb.readRegister(id, (uint16_t)0, last_register_addr+last_register_addr_length, getAllRegisteredData, &log);
          if (result == false)
          {
            Serial.println(log);
            return;
          }
          else
          {
            for (int index = 0; index < the_number_of_control_item; index++)
            {
              uint32_t data = 0;

              if (dxl_wb.getProtocolVersion() == 2.0f)
              {
                data = getAllRegisteredData[control_item[index].address];
                Serial.print("\t");
                Serial.print(control_item[index].item_name);
                Serial.print(" : ");
                Serial.println(data);
              }
              else if (dxl_wb.getProtocolVersion() == 1.0f)
              {
                switch (control_item[index].data_length)
                {
                  case BYTE:
                    data = getAllRegisteredData[control_item[index].address];
                    Serial.print("\t");
                    Serial.print(control_item[index].item_name);
                    Serial.print(" : ");
                    Serial.println(data);
                    break;

                  case WORD:
                    data = DXL_MAKEWORD(getAllRegisteredData[control_item[index].address], getAllRegisteredData[control_item[index].address+1]);
                    Serial.print("\t");
                    Serial.print(control_item[index].item_name);
                    Serial.print(" : ");
                    Serial.println(data);
                    break;

                  case DWORD:
                    data = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[control_item[index].address],   getAllRegisteredData[control_item[index].address+1]),
                                          DXL_MAKEWORD(getAllRegisteredData[control_item[index].address+2], getAllRegisteredData[control_item[index].address+3]));
                    Serial.print("\t");
                    Serial.print(control_item[index].item_name);
                    Serial.print(" : ");
                    Serial.println(data);
                    break;

                  default:
                    data = getAllRegisteredData[control_item[index].address];
                    break;
                } 
              }
            }
          }
        }
      }
      else if (cmd[0] == "sync_write_handler")
      {
        static uint8_t sync_write_handler_index = 0;
        uint8_t id = cmd[1].toInt();

        result = dxl_wb.addSyncWriteHandler(id, cmd[2].c_str(), &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to add sync write handler\n");
          return;
        }
        else
        {
          Serial.println(log);
          Serial.print("sync_write_handler_index = ");
          Serial.println(sync_write_handler_index);
        }
      }
      else if (cmd[0] == "sync_read_handler")
      {
        static uint8_t sync_read_handler_index = 0;
        uint8_t id = cmd[1].toInt();        

        result = dxl_wb.addSyncReadHandler(id, cmd[2].c_str(), &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to add sync write handler\n");
          return;
        }
        else
        {
          Serial.println(log);
          Serial.print("sync_read_handler_index = ");
          Serial.println(sync_read_handler_index);
        }
      }
      else if (cmd[0] == "bulk_write_handler")
      {
        result = dxl_wb.initBulkWrite(&log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to init bulk write handler\n");
          return;
        }
        else
          Serial.println(log);
      }
      else if (cmd[0] == "bulk_write_param")
      {
        uint8_t id = cmd[1].toInt();

        result = dxl_wb.addBulkWriteParam(id, cmd[2].c_str(), cmd[3].toInt(), &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to add param for bulk write\n");
          return;
        }
        else
          Serial.println(log);
      }
      else if (cmd[0] == "bulk_write")
      {
        result = dxl_wb.bulkWrite(&log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to bulk write\n");
          return;
        }
        else
          Serial.println(log);
      }
      else if (cmd[0] == "bulk_read_handler")
      {
        result = dxl_wb.initBulkRead(&log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to init bulk read handler\n");
          return;
        }
        else
          Serial.println(log);
      }
      else if (cmd[0] == "bulk_read_param")
      {
        uint8_t id = cmd[1].toInt();
        result = dxl_wb.addBulkReadParam(id, cmd[2].c_str(), &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.print("Failed to add param for bulk read\n");
          return;
        }
        else
          Serial.println(log);
      }
      else if (cmd[0] == "bulk_read")
      {
        result = dxl_wb.bulkRead(&log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to bulk read");
          return;
        }
        else
          printf("%s\n", log);

        int32_t get_data[dxl_wb.getTheNumberOfBulkReadParam()];
        result = dxl_wb.getBulkReadData(&get_data[0], &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to get bulk read data");
          return;
        }
        else
        {
          Serial.println(log);
          for (uint8_t index = 0; index < dxl_wb.getTheNumberOfBulkReadParam(); index++)
          {
            Serial.print("data[");
            Serial.print(index);
            Serial.print("] : ");
            Serial.print(get_data[index]);
          }
          Serial.println("");
        }

        dxl_wb.clearBulkReadParam();
      }
      else if (isAvailableID(cmd[1].toInt()) && isAvailableID(cmd[2].toInt()))
      {
        if (cmd[0] == "sync_write")
        {
          uint8_t id_1 = cmd[1].toInt();
          uint8_t id_2 = cmd[2].toInt();
          uint8_t id[2] = {id_1, id_2};
          uint8_t id_num = 2;

          int32_t data[2] = {0, 0};
          data[0] = cmd[4].toInt();
          data[1] = cmd[5].toInt();

          uint8_t handler_index = cmd[3].toInt();

          result = dxl_wb.syncWrite(handler_index, id, id_num, (int32_t *)data, 1, &log);
          if (result == false)
          {
            Serial.println(log);
            return;
          }
          else
            Serial.println(log);
        }
        else if (cmd[0] == "sync_read")
        {
          uint8_t id_1 = cmd[1].toInt();
          uint8_t id_2 = cmd[2].toInt();
          uint8_t id[2] = {id_1, id_2};
          uint8_t id_num = 2;

          int32_t data[2] = {0, 0};
          uint8_t handler_index = cmd[3].toInt();

          result = dxl_wb.syncRead(handler_index, id, id_num, &log);
          if (result == false)
          {
            Serial.println(log);
            return;
          }
          else
          {
            Serial.println(log);
          }

          result = dxl_wb.getSyncReadData(handler_index, id, id_num, data, &log);
          if (result == false)
          {
            Serial.println(log);
            return;
          }
          else
          {
            Serial.println(log);
            Serial.print("[ID ");
            Serial.print(cmd[1].toInt());
            Serial.print(" ]");
            Serial.print(" data : ");
            Serial.println(data[0]);
            Serial.print("[ID ");
            Serial.print(cmd[2].toInt());
            Serial.print(" ]");
            Serial.print(" data : ");
            Serial.println(data[0]);
          }
        }
      }
      else if (cmd[0] == "id")
      {
        uint8_t id     = cmd[1].toInt();
        uint8_t new_id = cmd[2].toInt();

        result = dxl_wb.changeID(id, new_id, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "baud")
      {
        uint8_t  id       = cmd[1].toInt();
        uint32_t  new_baud  = cmd[2].toInt();

        result = dxl_wb.changeBaudrate(id, new_baud, &log);
        if (result == false)
        {
          Serial.println(log);
          return ;
        }
        else
        {
          result = dxl_wb.setBaudrate(new_baud, &log);
          Serial.println(log);
        }
      }
      else if (cmd[0] == "torque_on")
      {
        uint8_t id       = cmd[1].toInt();

        result = dxl_wb.torqueOn(id, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "torque_off")
      {
        uint8_t id       = cmd[1].toInt();

        result = dxl_wb.torqueOff(id, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "joint")
      {
        uint8_t id    = cmd[1].toInt();
        uint16_t goal = cmd[2].toInt();

        result = dxl_wb.jointMode(id, 0, 0, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }

        result = dxl_wb.goalPosition(id, (int32_t)goal, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "wheel")
      {
        uint8_t id    = cmd[1].toInt();
        int32_t goal  = cmd[2].toInt();

        result = dxl_wb.wheelMode(id, 0, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }

        result = dxl_wb.goalVelocity(id, (int32_t)goal, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "write")
      {
        uint8_t id = cmd[1].toInt();      
        uint32_t value = cmd[3].toInt(); 

        result = dxl_wb.writeRegister(id, cmd[2].c_str(), value, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to write");
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "read")
      {
        uint8_t id = cmd[1].toInt();

        int32_t data = 0;
        
        result = dxl_wb.readRegister(id, cmd[2].c_str(), &data, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to read");
          return;
        }
        else
        {
          Serial.println(log);
          Serial.print("read data : ");
          Serial.println(data);
        }
      }
      else if (cmd[0] == "reboot")
      {
        uint8_t id = cmd[1].toInt();

        result = dxl_wb.reboot(id, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to reboot");
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else if (cmd[0] == "reset")
      {
        uint8_t id = cmd[1].toInt();

        result = dxl_wb.reset(id, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to reset");
          return;
        }
        else
        {
          Serial.println(log);
        }
      }
      else 
      {
        Serial.println("Wrong command");
      }
    }
    else 
    {
      Serial.println("Please check ID");
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

bool isAvailableID(uint8_t id)
{
  for (int dxl_cnt = 0; dxl_cnt < (scan_cnt + ping_cnt); dxl_cnt++)
  {
    if (get_id[dxl_cnt] == id)
      return true;
  }

  return false;
}

void printInst(void)
{
  Serial.print("-------------------------------------\n");
  Serial.print("Set begin before scan or ping\n");
  Serial.print("-------------------------------------\n");
  Serial.print("help\n");
  Serial.print("begin  (BAUD)\n");
  Serial.print("scan   (RANGE)\n");
  Serial.print("ping   (ID)\n");
  Serial.print("control_table (ID)\n");
  Serial.print("id     (ID) (NEW_ID)\n");
  Serial.print("baud   (ID) (NEW_BAUD)\n");
  Serial.print("torque_on (ID)\n");
  Serial.print("torque_off (ID)\n");
  Serial.print("joint  (ID) (GOAL_POSITION)\n");
  Serial.print("wheel  (ID) (GOAL_VELOCITY)\n");
  Serial.print("write  (ID) (ADDRESS_NAME) (DATA)\n");
  Serial.print("read   (ID) (ADDRESS_NAME)\n");
  Serial.print("sync_write_handler (Ref_ID) (ADDRESS_NAME)\n");
  Serial.print("sync_write (ID_1) (ID_2) (HANDLER_INDEX) (PARAM_1) (PARAM_2)\n");
  Serial.print("sync_read_handler (Ref_ID) (ADDRESS_NAME)\n");
  Serial.print("sync_read (ID_1) (ID_2) (HANDLER_INDEX)\n");
  Serial.print("bulk_write_handler\n");
  Serial.print("bulk_write_param (ID) (ADDRESS_NAME) (PARAM)\n");
  Serial.print("bulk_write\n");
  Serial.print("bulk_read_handler\n");
  Serial.print("bulk_read_param (ID) (ADDRESS_NAME)\n");
  Serial.print("bulk_read\n");
  Serial.print("reboot (ID) \n");
  Serial.print("reset  (ID) \n");
  Serial.print("end\n");
  Serial.print("-------------------------------------\n");
  Serial.print("Press Enter Key\n");
}