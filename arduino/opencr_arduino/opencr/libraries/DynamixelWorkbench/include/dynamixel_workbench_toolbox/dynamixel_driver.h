/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#ifndef DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_DRIVER_H

#include "dynamixel_tool.h"

#if defined(__OPENCR__) || defined(__OPENCM904__)
  #include <Arduino.h>
  #include <DynamixelSDK.h>
#elif defined(__linux__) || defined(__APPLE__)
  #include "unistd.h"
  #include "dynamixel_sdk/dynamixel_sdk.h"
#endif

#define MAX_DXL_SERIES_NUM  5
#define MAX_HANDLER_NUM     5
#define MAX_BULK_PARAMETER  20

typedef struct 
{
  const ControlItem *control_item; 
  dynamixel::GroupSyncWrite *groupSyncWrite;    
} SyncWriteHandler;

typedef struct 
{
  const ControlItem *control_item;
  dynamixel::GroupSyncRead  *groupSyncRead;     
} SyncReadHandler;

typedef struct
{
  uint8_t  id;
  uint16_t address;
  uint16_t data_length;
} BulkParameter;

typedef struct
{
  int dxl_comm_result;
  bool dxl_addparam_result;
  bool dxl_getdata_result;
  uint8_t dxl_error;
} ErrorFromSDK;

class DynamixelDriver
{
 private:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
  SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];

  dynamixel::GroupBulkRead  *groupBulkRead_;  
  dynamixel::GroupBulkWrite *groupBulkWrite_;
  BulkParameter bulk_read_param_[MAX_BULK_PARAMETER];
 
  DynamixelTool tools_[MAX_DXL_SERIES_NUM];

  uint8_t tools_cnt_;
  uint8_t sync_write_handler_cnt_;
  uint8_t sync_read_handler_cnt_;
  uint8_t bulk_read_parameter_cnt_;

 public:
  DynamixelDriver();
  ~DynamixelDriver();

  bool init(const char* device_name = "/dev/ttyUSB0", 
            uint32_t baud_rate = 57600, 
            const char **log = NULL);

  bool begin(const char* device_name = "/dev/ttyUSB0", 
            uint32_t baud_rate = 57600, 
            const char **log = NULL);

  bool setPortHandler(const char *device_name, const char **log = NULL);
  bool setBaudrate(uint32_t baud_rate, const char **log = NULL);
  bool setPacketHandler(float protocol_version, const char **log = NULL);

  float getProtocolVersion(void);
  uint32_t getBaudrate(void);

  const char * getModelName(uint8_t id, const char **log = NULL);
  uint16_t getModelNumber(uint8_t id, const char **log = NULL);
  const ControlItem *getControlTable(uint8_t id, const char **log = NULL);
  const ControlItem *getItemInfo(uint8_t id, const char *item_name, const char **log = NULL);
  uint8_t getTheNumberOfControlItem(uint8_t id, const char **log = NULL);
  const ModelInfo* getModelInfo(uint8_t id, const char **log = NULL);

  uint8_t getTheNumberOfSyncWriteHandler(void);
  uint8_t getTheNumberOfSyncReadHandler(void);
  uint8_t getTheNumberOfBulkReadParam(void);

  bool scan(uint8_t *get_id,
            uint8_t *get_the_number_of_id, 
            uint8_t range = 253,
            const char **log = NULL);

  bool scan(uint8_t *get_id,
            uint8_t *get_the_number_of_id, 
            uint8_t start_number,
            uint8_t end_number,
            const char **log = NULL);

  bool ping(uint8_t id, 
            uint16_t *get_model_number,
            const char **log = NULL);

  bool ping(uint8_t id,
            const char **log = NULL);

  bool clearMultiTurn(uint8_t id, const char **log = NULL);

  bool reboot(uint8_t id, const char **log = NULL);
  bool reset(uint8_t id, const char **log = NULL);

  bool writeRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t* data, const char **log = NULL);
  bool writeRegister(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

  bool writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log = NULL);
  bool writeOnlyRegister(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

  bool readRegister(uint8_t id, uint16_t address, uint16_t length, uint32_t *data, const char **log = NULL);
  bool readRegister(uint8_t id, const char *item_name, int32_t *data, const char **log = NULL);

  void getParam(int32_t data, uint8_t *param);

  bool addSyncWriteHandler(uint16_t address, uint16_t length, const char **log = NULL);
  bool addSyncWriteHandler(uint8_t id, const char *item_name, const char **log = NULL);

  bool syncWrite(uint8_t index, int32_t *data, const char **log = NULL);
  bool syncWrite(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, uint8_t data_num_for_each_id, const char **log = NULL);

  bool addSyncReadHandler(uint16_t address, uint16_t length, const char **log = NULL);
  bool addSyncReadHandler(uint8_t id, const char *item_name, const char **log = NULL);

  bool syncRead(uint8_t index, const char **log = NULL);
  bool syncRead(uint8_t index, uint8_t *id, uint8_t id_num, const char **log = NULL);

  bool getSyncReadData(uint8_t index, int32_t *data, const char **log = NULL);
  bool getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, const char **log = NULL);
  bool getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, uint16_t address, uint16_t length, int32_t *data, const char **log = NULL);

  bool initBulkWrite(const char **log = NULL);

  bool addBulkWriteParam(uint8_t id, uint16_t address, uint16_t length, int32_t data, const char **log = NULL);
  bool addBulkWriteParam(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

  bool bulkWrite(const char **log = NULL);

  bool initBulkRead(const char **log = NULL);

  bool addBulkReadParam(uint8_t id, uint16_t address, uint16_t length, const char **log = NULL);
  bool addBulkReadParam(uint8_t id, const char *item_name, const char **log = NULL);

  bool bulkRead(const char **log = NULL);

  bool getBulkReadData(int32_t *data, const char **log = NULL);
  bool getBulkReadData(uint8_t *id, uint8_t id_num, uint16_t *address, uint16_t *length, int32_t *data, const char **log = NULL);

  bool clearBulkReadParam(void);

 private:
  void initTools(void);
  bool setTool(uint16_t model_number, uint8_t id, const char **log = NULL);
  uint8_t getTool(uint8_t id, const char **log = NULL);
};

#endif //DYNAMIXEL_DRIVER_H
