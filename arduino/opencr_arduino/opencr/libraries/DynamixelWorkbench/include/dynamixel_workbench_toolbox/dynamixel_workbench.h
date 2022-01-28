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

#ifndef DYNAMIXEL_WORKBENCH_H_
#define DYNAMIXEL_WORKBENCH_H_

#include "dynamixel_driver.h"

class DynamixelWorkbench : public DynamixelDriver
{
 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool torque(uint8_t id, int32_t onoff, const char **log = NULL);
  bool torqueOn(uint8_t id, const char **log = NULL);
  bool torqueOff(uint8_t id, const char **log = NULL);

  bool changeID(uint8_t id, uint8_t new_id, const char **log = NULL);
  bool changeBaudrate(uint8_t id, uint32_t new_baudrate, const char **log = NULL);
  bool changeProtocolVersion(uint8_t id, uint8_t version, const char **log = NULL);

  bool itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);
  bool itemRead(uint8_t id, const char *item_name, int32_t *data, const char **log = NULL);

  bool led(uint8_t id, int32_t onoff, const char **log = NULL);
  bool ledOn(uint8_t id, const char **log = NULL);
  bool ledOff(uint8_t id, const char **log = NULL);

  bool setNormalDirection(uint8_t id, const char **log = NULL);
  bool setReverseDirection(uint8_t id, const char **log = NULL);
  
  bool setVelocityBasedProfile(uint8_t id, const char **log = NULL);
  bool setTimeBasedProfile(uint8_t id, const char **log = NULL);

  bool setSecondaryID(uint8_t id, uint8_t secondary_id, const char **log = NULL);

  bool setCurrentControlMode(uint8_t id, const char **log = NULL);
  bool setTorqueControlMode(uint8_t id, const char **log = NULL);
  bool setVelocityControlMode(uint8_t id, const char **log = NULL);  
  bool setPositionControlMode(uint8_t id, const char **log = NULL);  
  bool setExtendedPositionControlMode(uint8_t id, const char **log = NULL);
  bool setMultiTurnControlMode(uint8_t id, const char **log = NULL);
  bool setCurrentBasedPositionControlMode(uint8_t id, const char **log = NULL);
  bool setPWMControlMode(uint8_t id, const char **log = NULL);

  bool setOperatingMode(uint8_t id, uint8_t index, const char **log = NULL);

  bool jointMode(uint8_t id, int32_t velocity = 0, int32_t acceleration = 0, const char **log = NULL);
  bool wheelMode(uint8_t id, int32_t acceleration = 0, const char **log = NULL);
  bool currentBasedPositionMode(uint8_t id, int32_t current = 0, const char **log = NULL);

  bool goalPosition(uint8_t id, int value, const char **log = NULL);    //keep compatibility with older codes
  // bool goalPosition(uint8_t id, int32_t value, const char **log = NULL);
  bool goalPosition(uint8_t id, float radian, const char **log = NULL);

  bool goalSpeed(uint8_t id, int value, const char **log = NULL);       //keep compatibility with older codes
  bool goalVelocity(uint8_t id, int value, const char **log = NULL);    //keep compatibility with older codes  
  // bool goalVelocity(uint8_t id, int32_t value, const char **log = NULL);
  bool goalVelocity(uint8_t id, float velocity, const char **log = NULL);

  bool getPresentPositionData(uint8_t id, int32_t* data, const char **log = NULL);
  bool getRadian(uint8_t id, float* radian, const char **log = NULL);

  bool getPresentVelocityData(uint8_t id, int32_t* data, const char **log = NULL);
  bool getVelocity(uint8_t id, float* velocity, const char **log = NULL);

  int32_t convertRadian2Value(uint8_t id, float radian);
  float convertValue2Radian(uint8_t id, int32_t value);

  int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian);
  float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian);

  int32_t convertVelocity2Value(uint8_t id, float velocity);
  float convertValue2Velocity(uint8_t id, int32_t value);

  int16_t convertCurrent2Value(uint8_t id, float current);
  float convertValue2Current(uint8_t id, int16_t value);
  
  // This function will return incorrect value for some DYNAMIXEL like P or X330 series
  int16_t convertCurrent2Value(float current);
  // This function will return incorrect value for some DYNAMIXEL like P or X330 series
  float convertValue2Current(int16_t value);

  float convertValue2Load(int16_t value);
};

#endif /*DYNAMIXEL_WORKBENCH_H_*/
