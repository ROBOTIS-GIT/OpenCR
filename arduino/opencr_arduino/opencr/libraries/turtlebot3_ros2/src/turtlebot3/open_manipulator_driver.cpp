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

#include "../../include/turtlebot3/open_manipulator_driver.h"


/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_JOINT_1 = 11; 
const uint8_t DXL_MOTOR_ID_JOINT_2 = 12; 
const uint8_t DXL_MOTOR_ID_JOINT_3 = 13; 
const uint8_t DXL_MOTOR_ID_JOINT_4 = 14; 
const uint8_t DXL_MOTOR_ID_GRIPPER = 15; 

static ParamForSyncReadInst_t sync_read_param;
static ParamForSyncWriteInst_t sync_write_param;
static RecvInfoFromStatusInst_t read_result;



OpenManipulatorDriver::OpenManipulatorDriver(Dynamixel2Arduino &dxl_param)
: dxl(dxl_param),
  torque_(false),
  is_init_(false),
  is_connected_(false)
{
  motor_id_[JOINT_1] = DXL_MOTOR_ID_JOINT_1;
  motor_id_[JOINT_2] = DXL_MOTOR_ID_JOINT_2;
  motor_id_[JOINT_3] = DXL_MOTOR_ID_JOINT_3;
  motor_id_[JOINT_4] = DXL_MOTOR_ID_JOINT_4;
  motor_id_[GRIPPER] = DXL_MOTOR_ID_GRIPPER;
}

OpenManipulatorDriver::~OpenManipulatorDriver()
{
  close();
}

bool OpenManipulatorDriver::init(void)
{
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX;
  for (int i=0; i<JOINT_MOTOR_NUM_MAX; i++){
    sync_write_param.xel[i].id = motor_id_[i];
  }

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;
  for (int i=0; i<JOINT_MOTOR_NUM_MAX; i++){
    sync_read_param.xel[i].id = motor_id_[i];    
  }

  // Enable Dynamixel Torque
  set_torque(true);

  is_init_ = true;

  return true;
}

bool OpenManipulatorDriver::is_connected()
{
  bool ret = true;

  if (is_init_ == false) return false;

  for (int i=0; i<JOINT_MOTOR_NUM_MAX; i++) {
    if (dxl.ping(motor_id_[i]) == false) {
      ret = false;
      break;
    }
  }

  is_connected_ = ret;
  
  return ret;
}

bool OpenManipulatorDriver::is_ready(void)
{
  if (is_init_ == false) return false;
  if (is_connected_ == false) return false;

  return true;
}

bool OpenManipulatorDriver::set_torque(bool onoff)
{
  bool ret = false;
  
  if (is_ready() == false) return false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX;

  for (int i=0; i<JOINT_MOTOR_NUM_MAX; i++){
    sync_write_param.xel[i].data[0] = onoff;
  }

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool OpenManipulatorDriver::get_torque()
{
  bool ret = true;

  if (is_ready() == false) return false;

  for (int i=0; i<JOINT_MOTOR_NUM_MAX; i++) {
    if (dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, motor_id_[i]) == false) {
      ret = false;
      break;
    }
  }

  torque_ = ret;

  return torque_;
}

void OpenManipulatorDriver::close(void)
{
  if (is_ready() == false) return;

  // Disable Dynamixel Torque
  set_torque(false);
}

bool OpenManipulatorDriver::read_goal_position(joint_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 116;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&position_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_present_position(joint_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&position_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_present_velocity(joint_velocity_info_t &velocity_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&velocity_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_present_current(joint_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&current_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_profile_acceleration(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&accel_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_profile_velocity(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 112;
  sync_read_param.length = 4;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&accel_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::read_goal_current(joint_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_read_param.addr = 102;
  sync_read_param.length = 2;
  sync_read_param.id_count = JOINT_MOTOR_NUM_MAX;

  if(dxl.syncRead(sync_read_param, read_result)){
    for (int i=0; i<sync_read_param.id_count; i++) {
      memcpy(&current_info.value[i], read_result.xel[i].data, read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::write_goal_position_joint(joint_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_write_param.addr = 116;
  sync_write_param.length = 4;
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX - 1;

  for (int i=0; i<sync_write_param.id_count; i++) {
    memcpy(sync_write_param.xel[i].data, &position_info.value[i], sync_write_param.length);
  }

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::write_profile_acceleration_joint(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX - 1;

  for (int i=0; i<sync_write_param.id_count; i++) {
    memcpy(sync_write_param.xel[i].data, &accel_info.value[i], sync_write_param.length);
  }

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::write_profile_velocity_joint(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_write_param.addr = 112;
  sync_write_param.length = 4;
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX - 1;

  for (int i=0; i<sync_write_param.id_count; i++) {
    memcpy(sync_write_param.xel[i].data, &accel_info.value[i], sync_write_param.length);
  }

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::write_goal_current_joint(joint_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  sync_write_param.addr = 102;
  sync_write_param.length = 2;
  sync_write_param.id_count = JOINT_MOTOR_NUM_MAX - 1;

  for (int i=0; i<sync_write_param.id_count; i++) {
    memcpy(sync_write_param.xel[i].data, &current_info.value[i], sync_write_param.length);
  }

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool OpenManipulatorDriver::write_goal_position_gripper(joint_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  ret = dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, motor_id_[GRIPPER], position_info.value[GRIPPER]);

  return ret;
}

bool OpenManipulatorDriver::write_profile_acceleration_gripper(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  ret = dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, motor_id_[GRIPPER], accel_info.value[GRIPPER]);

  return ret;
}

bool OpenManipulatorDriver::write_profile_velocity_gripper(joint_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  ret = dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, motor_id_[GRIPPER], accel_info.value[GRIPPER]);

  return ret;
}

bool OpenManipulatorDriver::write_goal_current_gripper(joint_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  ret = dxl.writeControlTableItem(ControlTableItem::GOAL_CURRENT, motor_id_[GRIPPER], current_info.value[GRIPPER]);

  return ret;
}