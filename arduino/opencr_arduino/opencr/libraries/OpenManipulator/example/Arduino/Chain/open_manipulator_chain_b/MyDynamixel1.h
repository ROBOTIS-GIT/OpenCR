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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef MYDYNAMIXEL1_H_
#define MYDYNAMIXEL1_H_

#include <DynamixelWorkbench.h>
#include <vector>

#include <OMAPI.h>
#include <OMDebug.h>

#define DEVICE_NAME ""
#define OPERATING_MODE "Operating_Mode"
#define TORQUE_ENABLE "Torque_Enable"
#define GOAL_POSITION "Goal_Position"
#define PRESENT_POSITION "Present_Position"
#define PRESENT_CURRENT "Present_Current"

#define MAX_POSITION_LIMIT_ADDR 48
#define MIN_POSITION_LIMIT_ADDR 52
#define CW_ANGLE_LIMIT_ADDR 6
#define CCW_ANGLE_LIMIT_ADDR 8

#define MAX_POSITION_LIMIT_LENGTH 4
#define MIN_POSITION_LIMIT_LENGTH 4
#define CW_ANGLE_LIMIT_LENGTH 2
#define CCW_ANGLE_LIMIT_LENGTH 2

typedef struct
{
  uint8_t size;
  uint32_t baud_rate;
} DXL_INFO1;

namespace MY_DYNAMIXEL1
{
class Dynamixel : public OPEN_MANIPULATOR::Actuator
{
private:
  DynamixelWorkbench dxl_wb_;
  DXL_INFO1 dxl_info_;

  std::vector<uint8_t> dxl_id_;
  std::vector<float> radian_value_;
  std::vector<float> torque_value_;

public:
  Dynamixel(){};
  virtual ~Dynamixel(){};

  bool init(uint32_t baud_rate);
  bool setMode(uint8_t id, uint8_t mode);
  bool setPositionControlMode(uint8_t id, uint16_t profile_velocity = 0, uint16_t profile_acceleration = 0);
  bool setCurrentBasedPositionControlMode(uint8_t id, uint8_t current = 10);
  bool setMaxPositionLimit(uint8_t id, float radian);
  bool setMinPositionLimit(uint8_t id, float radian);

  void addSyncWriteHandler(const char *table_item);
  void addSyncReadHandler(const char *table_item);

  bool setEnable(uint8_t id);
  bool setDisable(uint8_t id);
  bool enableAllDynamixel();
  bool disableAllDynamixel();

  bool setAngle(std::vector<float> radian_vector);
  bool setAngle(std::vector<uint8_t> id, std::vector<float> radian_vector);
  bool setAngle(uint8_t id, float radian);
  std::vector<float> getAngle();
  std::vector<float> getCurrent();

  uint8_t getDynamixelSize();
  std::vector<uint8_t> getDynamixelIDs();
  uint32_t getBaudRate();
  int32_t getData(uint8_t id, uint16_t addr, uint8_t length);

  int32_t convertRadian2Value(uint8_t id, float radian);

  virtual void initActuator(const void *arg);
  virtual void setActuatorControlMode();
  virtual void Enable();
  virtual void Disable();
  virtual bool sendMultipleActuatorAngle(std::vector<uint8_t> actuator_id, std::vector<float> radian_vector);
  virtual bool sendAllActuatorAngle(std::vector<float> radian_vector);
  virtual bool sendActuatorAngle(uint8_t actuator_id, float radian);
  virtual bool sendActuatorSignal(uint8_t actuator_id, bool onoff);
  virtual std::vector<float> receiveAllActuatorAngle(void);
};
} // namespace MY_DYNAMIXEL1
#endif // MYDYNAMIXEL1_HPP_
