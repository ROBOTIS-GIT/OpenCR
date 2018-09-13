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

#ifndef OPEN_MANIPULATOR_MOTOR_DRIVER_H_
#define OPEN_MANIPULATOR_MOTOR_DRIVER_H_

#include <DynamixelWorkbench.h>

#define DXL_NUM                         5
#define JOINT_NUM                       4

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ON                   1       // Value for enabling the torque
#define TORQUE_OFF                  0       // Value for disabling the torque

#define JOINT1    11
#define JOINT2    12
#define JOINT3    13
#define JOINT4    14
#define GRIPPER   15

#define DEBUG_SERIAL  SerialBT2

class OpenManipulatorMotorDriver
{
 public:
  OpenManipulatorMotorDriver();
  ~OpenManipulatorMotorDriver();

  bool init(void);
  void closeDynamixel(void);
  bool setJointTorque(bool onoff);
  bool setGripperTorque(bool onoff);
  bool getJointTorque(void);
  bool getGripperTorque(void);
  bool setOperatingMode(uint8_t id, uint8_t operating_mode);
  bool readPosition(double *value);
  bool readVelocity(double *value);
  bool writeJointPosition(double *value);
  bool writeGripperPosition(double value);

 private:
  DynamixelWorkbench joint_controller_;
  DynamixelWorkbench gripper_controller_;
  uint8_t dxl_id_[JOINT_NUM];
  float protocol_version_;

  bool joint_torque_state_;
  bool gripper_torque_state_;
};

#endif // OPEN_MANIPULATOR_MOTOR_DRIVER_H_
