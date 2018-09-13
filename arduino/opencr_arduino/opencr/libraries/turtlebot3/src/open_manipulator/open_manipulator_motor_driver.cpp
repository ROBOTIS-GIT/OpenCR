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

/* Authors: Darby Lim */

#include "../../include/open_manipulator/open_manipulator_motor_driver.h"

OpenManipulatorMotorDriver::OpenManipulatorMotorDriver()
{
  dxl_id_[0] = JOINT1;
  dxl_id_[1] = JOINT2;
  dxl_id_[2] = JOINT3;
  dxl_id_[3] = JOINT4;
}

OpenManipulatorMotorDriver::~OpenManipulatorMotorDriver()
{
  closeDynamixel();
}

bool OpenManipulatorMotorDriver::init(void)
{
  DEBUG_SERIAL.begin(57600);

  bool joint_controller_state = false;
  bool gripper_controller_state = false;

  joint_controller_state   = joint_controller_.begin(DEVICENAME, BAUDRATE);
  gripper_controller_state = gripper_controller_.begin(DEVICENAME, BAUDRATE);

  if (joint_controller_state == false)
    DEBUG_SERIAL.println("Failed to open port(joint controller)");
  else if (gripper_controller_state == false)
    DEBUG_SERIAL.println("Failed to open port(gripper controller)");

  uint16_t get_model_number;
  for (int num = 0; num < JOINT_NUM; num++)
  {
    joint_controller_state = joint_controller_.ping(dxl_id_[num], &get_model_number);
    if (joint_controller_state == false)
      DEBUG_SERIAL.println("Failed to ping(joint controller)");

    joint_controller_.jointMode(dxl_id_[num]);
  }

  gripper_controller_state = gripper_controller_.ping(GRIPPER, &get_model_number);
  if (gripper_controller_state == false)
    DEBUG_SERIAL.println("Failed to ping(gripper controller)");

  protocol_version_ = joint_controller_.getProtocolVersion();  

  if (protocol_version_ == 2.0)
    gripper_controller_.currentMode(GRIPPER, 30);
  else
    gripper_controller_.jointMode(GRIPPER);

  joint_controller_.addSyncWrite("Goal_Position");

  if (protocol_version_ == 2.0)
  {
    joint_controller_.addSyncRead("Present_Position");
    joint_controller_.addSyncRead("Present_Velocity");
  }

  double init_joint_position[4] = {0.0, -1.5707, 1.37, 0.2258};
  writeJointPosition(init_joint_position);
  writeGripperPosition(0.0);

  joint_torque_state_   = true;
  gripper_torque_state_ = true;

  if (joint_controller_state && gripper_controller_state)
    DEBUG_SERIAL.println("Success to init OpenManipulator Motor Driver(joint and gripper controller)");

  return true;
}

void OpenManipulatorMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setJointTorque(false);
  setGripperTorque(false);
}

bool OpenManipulatorMotorDriver::setJointTorque(bool onoff)
{ 
  for (int num = 0; num < JOINT_NUM; num++)  
    joint_controller_.itemWrite(dxl_id_[num], "Torque_Enable", onoff);

  joint_torque_state_ = onoff;
}

bool OpenManipulatorMotorDriver::getJointTorque()
{
  return joint_torque_state_;
}

bool OpenManipulatorMotorDriver::setGripperTorque(bool onoff)
{ 
  gripper_controller_.itemWrite(GRIPPER, "Torque_Enable", onoff);

  gripper_torque_state_ = onoff;
}

bool OpenManipulatorMotorDriver::getGripperTorque()
{
  return gripper_torque_state_;
}

bool OpenManipulatorMotorDriver::readPosition(double *value)
{
  int32_t get_joint_present_position[DXL_NUM];
  int32_t *get_position_ptr = NULL;

  if (protocol_version_ == 2.0)
  {
    get_position_ptr = joint_controller_.syncRead("Present_Position");

    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_position[index] = get_position_ptr[index];
  }
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_position[index] = joint_controller_.itemRead(index, "Present_Position");
  }

  int32_t get_gripper_present_position = gripper_controller_.itemRead(GRIPPER, "Present_Position");

  get_joint_present_position[DXL_NUM-1] = get_gripper_present_position;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller_.convertValue2Radian(index, get_joint_present_position[index]);
  }

  value[DXL_NUM-1] = gripper_controller_.convertValue2Radian(GRIPPER, get_joint_present_position[DXL_NUM-1]);
}

bool OpenManipulatorMotorDriver::readVelocity(double *value)
{
  int32_t get_joint_present_velocity[DXL_NUM];
  int32_t *get_velocity_ptr = NULL;

  if (protocol_version_ == 2.0)
  {
    get_velocity_ptr = joint_controller_.syncRead("Present_Velocity");

    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_velocity[index] = get_velocity_ptr[index];
  }
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < JOINT_NUM; index++)
      get_joint_present_velocity[index] = joint_controller_.itemRead(index, "Present_Velocity");
  }

  int32_t get_gripper_present_velocity = gripper_controller_.itemRead(GRIPPER, "Present_Velocity");

  get_joint_present_velocity[DXL_NUM-1] = get_gripper_present_velocity;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller_.convertValue2Velocity(index, get_joint_present_velocity[index]);
  }

  value[DXL_NUM-1] = gripper_controller_.convertValue2Velocity(GRIPPER, get_joint_present_velocity[DXL_NUM-1]);
}

bool OpenManipulatorMotorDriver::writeJointPosition(double *value)
{
  int32_t goal_position[JOINT_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
  {
    goal_position[index] = joint_controller_.convertRadian2Value(dxl_id_[index], value[index]);
  }

  return joint_controller_.syncWrite("Goal_Position", goal_position);
}

bool OpenManipulatorMotorDriver::writeGripperPosition(double value)
{
  return gripper_controller_.itemWrite(GRIPPER, "Goal_Position", gripper_controller_.convertRadian2Value(GRIPPER, value));
}