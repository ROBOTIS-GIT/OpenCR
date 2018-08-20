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

#ifndef DYNAMIXEL_WORKBENCH_H_
#define DYNAMIXEL_WORKBENCH_H_

#include "dynamixel_driver.h"

#define X_SERIES_CURRENT_CONTROL_MODE                  0
#define X_SERIES_VELOCITY_CONTROL_MODE                 1
#define X_SERIES_POSITION_CONTROL_MODE                 3
#define X_SERIES_EXTENDED_POSITION_CONTROL_MODE        4
#define X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE   5
#define X_SERIES_VOLTAGE_CONTROL_MODE                  16

#define PRO_SERIES_TORQUE_CONTROL_MODE                 0
#define PRO_SERIES_VELOCITY_CONTROL_MODE               1
#define PRO_SERIES_POSITION_CONTROL_MODE               3
#define PRO_SERIES_EXTENDED_POSITION_CONTROL_MODE      4

class DynamixelWorkbench
{
 private:
  DynamixelDriver driver_;
  char dxl_[64];

 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool begin(const char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600);
 
  bool scan(uint8_t *get_id, uint8_t *get_id_num = 0, uint8_t range = 200);
  bool ping(uint8_t id, uint16_t *get_model_number = 0);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool setID(uint8_t id, uint8_t new_id);
  bool setBaud(uint8_t id, uint32_t new_baud);
  bool setPacketHandler(float protocol_version);

  float getProtocolVersion();
  char* getModelName(uint8_t id);

  bool ledOn(uint8_t id);
  bool ledOff(uint8_t id);

  bool jointMode(uint8_t id, uint16_t vel = 0, uint16_t acc = 0);
  bool wheelMode(uint8_t id, uint16_t vel = 0, uint16_t acc = 0);
  bool currentMode(uint8_t id, uint8_t cur = 50);

  bool goalPosition(uint8_t id, int32_t goal);
  bool goalSpeed(uint8_t id, int32_t goal);

  bool itemWrite(uint8_t id, const char* item_name, int32_t value);  // write value to item
  bool itemWrite(uint8_t id, uint16_t addr, uint8_t length, int32_t data);
  bool syncWrite(const char *item_name, int32_t* value);             // sync write
  bool syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data);
  bool bulkWrite(void);                                              // bulk write

  int32_t  itemRead(uint8_t id, const char* item_name);  // read value from item
  int32_t  itemRead(uint8_t id, uint16_t addr, uint8_t length);
  int32_t* syncRead(const char* item_name);              // sync read
  int32_t  bulkRead(uint8_t id, const char* item_name);  // bulk read

  void addSyncWrite(const char* item_name);
  void addSyncRead(const char* item_name);

  void initBulkWrite();
  void initBulkRead();

  bool addBulkWriteParam(uint8_t id, const char *item_name, int32_t data);
  bool addBulkReadParam(uint8_t id, const char *item_name);
  bool setBulkRead();

  int32_t convertRadian2Value(uint8_t id, float radian);
  float convertValue2Radian(uint8_t id, int32_t value);

  int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);
  float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);


  int32_t convertVelocity2Value(uint8_t id, float velocity);
  float convertValue2Velocity(uint8_t id, int32_t value);

  int16_t convertTorque2Value(uint8_t id, float torque);
  float convertValue2Torque(uint8_t id, int16_t value);

 private:
  void millis(uint16_t msec);

  bool torque(uint8_t id, bool onoff);
  bool setPositionControlMode(uint8_t id);
  bool setVelocityControlMode(uint8_t id);
  bool setCurrentControlMode(uint8_t id);
};

#endif /*DYNAMIXEL_WORKBENCH_H_*/
