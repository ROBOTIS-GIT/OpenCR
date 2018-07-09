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
  uint8_t* id_ptr;
  uint8_t size;
  float* position_ptr;
  float* current_ptr;
}DXL_INFO;

template <uint8_t DXL_SIZE, uint32_t BAUD_RATE>
class OMDynamixel
{
 private:
  DynamixelWorkbench dxl_wb_;
  DXL_INFO dxl_info_;

 public:
  OMDynamixel();
  ~OMDynamixel();

  bool init();

  bool setMode(uint8_t id, uint8_t mode);
  bool setPositionControlMode(uint8_t id);
  bool setCurrentBasedPositionControlMode(uint8_t id, uint8_t current);

  bool setEnable(uint8_t id);
  bool setDisable(uint8_t id);

  bool enableAllDynamixel();
  bool disableAllDynamixel();

  bool setAngle(float *data);
  bool setAngle(uint8_t id, float data);

  uint8_t getDynamixelSize();
  uint8_t* getDynamixelIds();
  uint32_t getBaudRate();

  float* getAngle();
  float* getCurrent();
};

#endif // OMDYNAMIXEL_H_