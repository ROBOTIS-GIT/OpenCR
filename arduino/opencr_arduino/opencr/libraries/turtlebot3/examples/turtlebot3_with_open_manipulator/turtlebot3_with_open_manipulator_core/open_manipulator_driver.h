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

#ifndef OPEN_MANIPULATOR_DRIVER_H_
#define OPEN_MANIPULATOR_DRIVER_H_

#include <DynamixelWorkbench.h>

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define JOINT_ID_1                       11
#define JOINT_ID_2                       12
#define JOINT_ID_3                       13
#define JOINT_ID_4                       14
#define JOINT_CNT                        4

#define GRIPPER_ID_1                     15
#define GRIPPER_CNT                      1

#define DEBUG_SERIAL  SerialBT2

typedef struct
{
  uint8_t id[20];
  uint8_t cnt;
} Dynamixel;

class OpenManipulatorDriver
{
 public:
  OpenManipulatorDriver();
  ~OpenManipulatorDriver();

  bool init(uint8_t *joint_id, uint8_t joint_cnt, uint8_t *gripper_id, uint8_t gripper_cnt);
  void closeDynamixel(void);
  bool setTorque(bool onoff);
  bool getTorqueState(void);
  bool syncReadDynamixelInfo(void);
  bool getPosition(double *get_data);
  bool getVelocity(double *get_data);
  bool getCurrent(double *get_data);
  bool writeJointPosition(double *set_data);
  bool writeJointProfileControlParam(double set_time, double acc = 0.0f);
  bool writeGripperPosition(double *set_data);
  bool writeGripperProfileControlParam(double set_time);

 private:
  DynamixelWorkbench dxl_wb_;

  Dynamixel joint_;
  Dynamixel gripper_;

  bool torque_state_;
};

#endif // OPEN_MANIPULATOR_DRIVER_H_
