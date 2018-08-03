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

#ifndef OMDYNAMIXEL_H_
#define OMDYNAMIXEL_H_
#if 0
#include <DynamixelWorkbench.h>

#define DEVICE_NAME       ""
#define OPERATING_MODE    "Operating_Mode"
#define TORQUE_ENABLE     "Torque_Enable"
#define GOAL_POSITION     "Goal_Position"
#define PRESENT_POSITION  "Present_Position"
#define PRESENT_CURRENT   "Present_Current"

typedef struct
{
  uint32_t baud_rate;
  uint8_t* id_ptr = NULL;
  uint8_t size;
  float* position_ptr = NULL;
  float* current_ptr = NULL;
}DXL_INFO;

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
class OMDynamixel
{
 private:
  DynamixelWorkbench dxl_wb_;
  DXL_INFO dxl_info_;

  uint8_t dxl_id_[DXL_SIZE];
  float radian_value_[DXL_SIZE];
  float torque_value_[DXL_SIZE];

 public:
  OMDynamixel();
  ~OMDynamixel(){};

  bool init();
  bool setMode(uint8_t id, uint8_t mode);
  bool setPositionControlMode(uint8_t id);
  bool setCurrentBasedPositionControlMode(uint8_t id, uint8_t current=10);
  bool setEnable(uint8_t id);
  bool setDisable(uint8_t id);
  bool enableAllDynamixel();
  bool disableAllDynamixel();
  bool setAngle(float *radian);
  bool setAngle(uint8_t id, float radian);
  uint8_t* getID();
  uint8_t getDynamixelSize();
  uint8_t* getDynamixelIds();
  uint32_t getBaudRate();
  int32_t getData(uint8_t id, const char* table_item);
  float* getAngle();
  float* getCurrent();
  void addSyncWriteHandler(const char* table_item);
  void addSyncReadHandler(const char* table_item);
};
#endif
#endif // OMDYNAMIXEL_HPP_
