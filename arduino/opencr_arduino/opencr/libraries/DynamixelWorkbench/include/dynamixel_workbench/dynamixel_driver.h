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

#ifndef DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H

#include "dynamixel_tool.h"

#if defined(__OPENCR__) || defined(__OPENCM904__)
  #include <DynamixelSDK.h>
#elif defined(__linux__)
  #include "dynamixel_sdk/dynamixel_sdk.h"
#endif

#define DXL_NUM 16
#define MAX_HANDLER 5

#define BYTE  1
#define WORD  2
#define DWORD 4

#define DEBUG false

typedef struct 
{
  ControlTableItem *cti; 
  dynamixel::GroupSyncWrite *groupSyncWrite;    
} SyncWriteHandler;

typedef struct 
{
  ControlTableItem *cti;
  dynamixel::GroupSyncRead  *groupSyncRead;     
} SyncReadHandler;

class DynamixelDriver
{
 private:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  dynamixel::PacketHandler *packetHandler_1;
  dynamixel::PacketHandler *packetHandler_2;

  SyncWriteHandler syncWriteHandler_[MAX_HANDLER];
  SyncReadHandler  syncReadHandler_[MAX_HANDLER];

  dynamixel::GroupBulkRead  *groupBulkRead_;  
  dynamixel::GroupBulkWrite *groupBulkWrite_;  
 
  DynamixelTool tools_[DXL_NUM];

  uint8_t tools_cnt_;
  uint8_t sync_write_handler_cnt_;
  uint8_t sync_read_handler_cnt_;

  char dxl_[64];

 public:
  DynamixelDriver();
  ~DynamixelDriver();

  bool begin(char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600);

  void setPortHandler(char *device_name, bool *error);
  void setPacketHandler(bool *error);
  void setPacketHandler(float protocol_version);
  void setBaudrate(uint32_t baud_rate, bool *error);

  float getProtocolVersion();
  char* getModelName(uint8_t id);

  uint8_t  scan(uint8_t *get_id, uint8_t num = 200, float protocol_version = 0.0);
  uint16_t ping(uint8_t id, float protocol_version = 0.0);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool writeRegister(uint8_t id, char *item_name, int32_t data);
  bool readRegister(uint8_t id, char *item_name, int32_t *data);

  void addSyncWrite(char *item_name);
  bool syncWrite(char *item_name, int32_t *data);

  void addSyncRead(char *item_name);
  bool syncRead(char *item_name, int32_t *data);

  void initBulkWrite();
  bool addBulkWriteParam(uint8_t id, char *item_name, int32_t data);
  bool bulkWrite();

  void initBulkRead();
  bool addBulkReadParam(uint8_t id, char *item_name);
  bool sendBulkReadPacket();
  bool bulkRead(uint8_t id, char *item_name, int32_t *data);

  int32_t convertRadian2Value(int8_t id, float radian);
  float convertValue2Radian(int8_t id, int32_t value);

 private:
  void setTools(uint16_t model_num, uint8_t id);
  uint8_t findTools(uint8_t id);
  uint8_t theNumberOfTools();
};

#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
