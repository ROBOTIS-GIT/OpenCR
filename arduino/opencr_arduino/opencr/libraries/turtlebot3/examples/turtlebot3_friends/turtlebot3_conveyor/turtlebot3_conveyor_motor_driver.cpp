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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "turtlebot3_conveyor_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(WHEEL_L_R, true);
  setTorque(WHEEL_R_R, true);
  setTorque(WHEEL_L_F, true);
  setTorque(WHEEL_R_F, true);

  setTorque(JOINT_L_R, true);
  setTorque(JOINT_R_R, true);
  setTorque(JOINT_L_F, true);
  setTorque(JOINT_R_F, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  return true;
}

bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

void Turtlebot3MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(WHEEL_L_R, false);
  setTorque(WHEEL_R_R, false);
  setTorque(WHEEL_L_F, false);
  setTorque(WHEEL_R_F, false);

  setTorque(JOINT_L_R, false);
  setTorque(JOINT_R_R, false);
  setTorque(JOINT_L_F, false);
  setTorque(JOINT_R_F, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MotorDriver::controlJoints(int32_t *value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_L_R, (uint8_t*)&value[0]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_R_R, (uint8_t*)&value[1]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_L_F, (uint8_t*)&value[2]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWritePosition_->addParam(JOINT_R_F, (uint8_t*)&value[3]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

bool Turtlebot3MotorDriver::controlWheels(int32_t *value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_L_R, (uint8_t*)&value[0]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_R_R, (uint8_t*)&value[1]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_L_F, (uint8_t*)&value[2]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(WHEEL_R_F, (uint8_t*)&value[3]);
  if (dxl_addparam_result_ != true)
    return false;

  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}
