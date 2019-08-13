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

#include "../../include/turtlebot3/turtlebot3_motor_driver.h"

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
const int OPENCR_DXL_DIR_PIN = 84;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: left_wheel_id_(DXL_LEFT_ID),
  right_wheel_id_(DXL_RIGHT_ID),
  torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);

  sync_write_param.id_count = 2;
  sync_write_param.xel[LEFT].id = left_wheel_id_;
  sync_write_param.xel[RIGHT].id = right_wheel_id_;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 2;
  sync_read_param.xel[LEFT].id = left_wheel_id_;
  sync_read_param.xel[RIGHT].id = right_wheel_id_;

  // Enable Dynamixel Torque
  setTorque(true);

  return true;
}

bool Turtlebot3MotorDriver::setTorque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[LEFT].data[0] = onoff;
  sync_write_param.xel[RIGHT].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool Turtlebot3MotorDriver::getTorque()
{
  if(dxl.readControlTableItem(TORQUE_ENABLE, left_wheel_id_) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, right_wheel_id_) == true){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);
}

bool Turtlebot3MotorDriver::readPresentPosition(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::readPresentVelocity(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::readPresentCurrent(int16_t &left_value, int16_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::readProfileAcceleration(uint32_t &left_value, uint32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::writeVelocity(int32_t left_value, int32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::writeProfileAcceleration(uint32_t left_value, uint32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::controlMotor(const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = writeVelocity((int32_t)wheel_velocity_cmd[LEFT], (int32_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
