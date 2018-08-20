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
  #include <Arduino.h>
  #include <DynamixelSDK.h>
#elif defined(__linux__)
  #include "stdio.h"
  #include "unistd.h"
  #include "dynamixel_sdk/dynamixel_sdk.h"
#endif

#define MAX_DXL_SERIES_NUM 5
#define MAX_HANDLER_NUM 5

#define BYTE  1
#define WORD  2
#define DWORD 4

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

  SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
  SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];

  dynamixel::GroupBulkRead  *groupBulkRead_;  
  dynamixel::GroupBulkWrite *groupBulkWrite_;  
 
  DynamixelTool tools_[MAX_DXL_SERIES_NUM];

  uint8_t tools_cnt_;
  uint8_t sync_write_handler_cnt_;
  uint8_t sync_read_handler_cnt_;

 public:
  DynamixelDriver();
  ~DynamixelDriver();

  bool init(const char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600);

  bool setPortHandler(const char *device_name);
  bool setPacketHandler(void);
  bool setPacketHandler(float protocol_version);
  bool setBaudrate(uint32_t baud_rate);

  float getProtocolVersion(void);
  int getBaudrate(void);
  char* getModelName(uint8_t id);
  uint16_t getModelNum(uint8_t id);
  ControlTableItem* getControlItemPtr(uint8_t id);
  uint8_t getTheNumberOfItem(uint8_t id);

  bool scan(uint8_t *get_id, uint8_t *get_id_num, uint8_t range = 200);
  bool ping(uint8_t id, uint16_t *get_model_number);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool writeRegister(uint8_t id, const char *item_name, int32_t data);
  bool writeRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t data);
  bool readRegister(uint8_t id, const char *item_name, int32_t *data);
  bool readRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t *data);
  bool readRegister(uint8_t id, uint16_t length, uint8_t *data);

  void addSyncWrite(const char *item_name);
  bool syncWrite(const char *item_name, int32_t *data);
  bool syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data);

  void addSyncRead(const char *item_name);
  bool syncRead(const char *item_name, int32_t *data);

  void initBulkWrite();
  bool addBulkWriteParam(uint8_t id, const char *item_name, int32_t data);
  bool bulkWrite();

  void initBulkRead();
  bool addBulkReadParam(uint8_t id, const char *item_name);
  bool sendBulkReadPacket();
  bool bulkRead(uint8_t id, const char *item_name, int32_t *data);

  int32_t convertRadian2Value(uint8_t id, float radian);
  float convertValue2Radian(uint8_t id, int32_t value);

  int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);
  float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);

  int32_t convertVelocity2Value(uint8_t id, float velocity);
  float convertValue2Velocity(uint8_t id, int32_t value);

  int16_t convertTorque2Value(uint8_t id, float torque);
  float convertValue2Torque(uint8_t id, int16_t value);

 private:
  void initDXLinfo(void);
  void setTools(uint16_t model_number, uint8_t id);
  const char *findModelName(uint16_t model_num);
  uint8_t getToolsFactor(uint8_t id);

  void millis(uint16_t msec);
};

#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
