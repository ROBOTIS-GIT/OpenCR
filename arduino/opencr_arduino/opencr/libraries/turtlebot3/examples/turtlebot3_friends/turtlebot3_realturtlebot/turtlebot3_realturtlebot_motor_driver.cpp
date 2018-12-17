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

#include "turtlebot3_realturtlebot_motor_driver.h"

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
  setTorque(DXL_LEG_LEFT_REAR_ID, true);
  setTorque(DXL_LEG_RIGHT_REAR_ID, true);
  setTorque(DXL_LEG_LEFT_FRONT_ID, true);
  setTorque(DXL_LEG_RIGHT_FRONT_ID, true);
  setTorque(DXL_SHOULDER_LEFT_REAR_ID, true);
  setTorque(DXL_SHOULDER_RIGHT_REAR_ID, true);
  setTorque(DXL_SHOULDER_LEFT_FRONT_ID, true);
  setTorque(DXL_SHOULDER_RIGHT_FRONT_ID, true);
  setTorque(DXL_HEAD_YAW_ID, true);
  setTorque(DXL_HEAD_PITCH_ID, true);

  // setProfileVelocity(front_joint_id_, 120); //TODO : precise calculation

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

bool Turtlebot3MotorDriver::setProfileAcceleration(uint8_t id, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_ACCELERATION, value, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

bool Turtlebot3MotorDriver::setProfileVelocity(uint8_t id, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_PROFILE_VELOCITY, value, &dxl_error);
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
  setTorque(DXL_LEG_LEFT_REAR_ID, false);
  setTorque(DXL_LEG_RIGHT_REAR_ID, false);
  setTorque(DXL_LEG_LEFT_FRONT_ID, false);
  setTorque(DXL_LEG_RIGHT_FRONT_ID, false);
  setTorque(DXL_SHOULDER_LEFT_REAR_ID, false);
  setTorque(DXL_SHOULDER_RIGHT_REAR_ID, false);
  setTorque(DXL_SHOULDER_LEFT_FRONT_ID, false);
  setTorque(DXL_SHOULDER_RIGHT_FRONT_ID, false);
  setTorque(DXL_HEAD_YAW_ID, false);
  setTorque(DXL_HEAD_PITCH_ID, false);

  // Close port
  portHandler_->closePort();
}

void Turtlebot3MotorDriver::syncWrite(int address, int length, int value)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, address, length);

  // Add Dynamixels goal values to the Syncwrite storage
  groupSyncWrite.addParam(DXL_LEG_LEFT_REAR_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_LEG_RIGHT_REAR_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_LEG_LEFT_FRONT_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_LEG_RIGHT_FRONT_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_SHOULDER_LEFT_REAR_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_SHOULDER_RIGHT_REAR_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_SHOULDER_LEFT_FRONT_ID, (uint8_t*)&value);
  groupSyncWrite.addParam(DXL_SHOULDER_RIGHT_FRONT_ID, (uint8_t*)&value);

  // Syncwrite goal position
  groupSyncWrite.txPacket();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void Turtlebot3MotorDriver::syncWrite(int address, int length, int* value)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, address, length);

  // Add Dynamixels goal values to the Syncwrite storage
  groupSyncWrite.addParam(DXL_LEG_LEFT_REAR_ID, (uint8_t*)&value[0]);
  groupSyncWrite.addParam(DXL_LEG_RIGHT_REAR_ID, (uint8_t*)&value[1]);
  groupSyncWrite.addParam(DXL_LEG_LEFT_FRONT_ID, (uint8_t*)&value[2]);
  groupSyncWrite.addParam(DXL_LEG_RIGHT_FRONT_ID, (uint8_t*)&value[3]);
  groupSyncWrite.addParam(DXL_SHOULDER_LEFT_REAR_ID, (uint8_t*)&value[4]);
  groupSyncWrite.addParam(DXL_SHOULDER_RIGHT_REAR_ID, (uint8_t*)&value[5]);
  groupSyncWrite.addParam(DXL_SHOULDER_LEFT_FRONT_ID, (uint8_t*)&value[6]);
  groupSyncWrite.addParam(DXL_SHOULDER_RIGHT_FRONT_ID, (uint8_t*)&value[7]);

  // Syncwrite goal position
  groupSyncWrite.txPacket();

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void Turtlebot3MotorDriver::syncRead(int address, int length, int* readValues)
{
  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, address, length);

  // Add parameter storage for Dynamixel#1 present position value
  groupSyncRead.addParam(DXL_LEG_LEFT_REAR_ID);
  groupSyncRead.addParam(DXL_LEG_RIGHT_REAR_ID);
  groupSyncRead.addParam(DXL_LEG_LEFT_FRONT_ID);
  groupSyncRead.addParam(DXL_LEG_RIGHT_FRONT_ID);
  groupSyncRead.addParam(DXL_SHOULDER_LEFT_REAR_ID);
  groupSyncRead.addParam(DXL_SHOULDER_RIGHT_REAR_ID);
  groupSyncRead.addParam(DXL_SHOULDER_LEFT_FRONT_ID);
  groupSyncRead.addParam(DXL_SHOULDER_RIGHT_FRONT_ID);

  groupSyncRead.txRxPacket();

  readValues[0] = (int)groupSyncRead.getData(DXL_LEG_LEFT_REAR_ID, address, length);
  readValues[1] = (int)groupSyncRead.getData(DXL_LEG_RIGHT_REAR_ID, address, length);
  readValues[2] = (int)groupSyncRead.getData(DXL_LEG_LEFT_FRONT_ID, address, length);
  readValues[3] = (int)groupSyncRead.getData(DXL_LEG_RIGHT_FRONT_ID, address, length);
  readValues[4] = (int)groupSyncRead.getData(DXL_SHOULDER_LEFT_REAR_ID, address, length);
  readValues[5] = (int)groupSyncRead.getData(DXL_SHOULDER_RIGHT_REAR_ID, address, length);
  readValues[6] = (int)groupSyncRead.getData(DXL_SHOULDER_LEFT_FRONT_ID, address, length);
  readValues[7] = (int)groupSyncRead.getData(DXL_SHOULDER_RIGHT_FRONT_ID, address, length);

  groupSyncRead.clearParam();
}
