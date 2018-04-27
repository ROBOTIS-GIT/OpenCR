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
  joint_controller.begin(DEVICENAME, BAUDRATE);
  gripper_controller.begin(DEVICENAME, BAUDRATE);

  for (int num = 0; num < JOINT_NUM; num++)
  {
    joint_controller.ping(dxl_id_[num]);
    joint_controller.jointMode(dxl_id_[num]);
  }

  gripper_controller.ping(GRIPPER);
  gripper_controller.currentMode(GRIPPER, 30);

  joint_controller.addSyncWrite("Goal_Position");
  joint_controller.addSyncRead("Present_Position");
  joint_controller.addSyncRead("Present_Velocity");

  return true;
}

void OpenManipulatorMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  for (int num = 0; num < JOINT_NUM; num++)  
    joint_controller.itemWrite(dxl_id_[num], "Torque_Enable", false);

  gripper_controller.itemWrite(GRIPPER, "Torque_Enable", false);
}

bool OpenManipulatorMotorDriver::setJointTorque(bool onoff)
{ 
  for (int num = 0; num < JOINT_NUM; num++)  
    joint_controller.itemWrite(dxl_id_[num], "Torque_Enable", onoff);
}

bool OpenManipulatorMotorDriver::setGripperTorque(bool onoff)
{ 
  gripper_controller.itemWrite(GRIPPER, "Torque_Enable", onoff);
}

bool OpenManipulatorMotorDriver::readPosition(double *value)
{
  int32_t* get_joint_present_position = joint_controller.syncRead("Present_Position");
  int32_t get_gripper_present_position = gripper_controller.itemRead(GRIPPER, "Present_Position");
  int32_t present_position[DXL_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_position[index] = get_joint_present_position[index];

  present_position[DXL_NUM-1] = get_gripper_present_position;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller.convertValue2Radian(dxl_id_[index], present_position[index]);
  }

  value[DXL_NUM-1] = gripper_controller.convertValue2Radian(GRIPPER, present_position[DXL_NUM-1]);
}

bool OpenManipulatorMotorDriver::readVelocity(double *value)
{
  int32_t* get_joint_present_velocity = joint_controller.syncRead("Present_Velocity");
  int32_t get_gripper_present_velocity = gripper_controller.itemRead(GRIPPER, "Present_Velocity");
  int32_t present_velocity[DXL_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
    present_velocity[index] = get_joint_present_velocity[index];

  present_velocity[DXL_NUM-1] = get_gripper_present_velocity;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    value[index] = joint_controller.convertValue2Velocity(dxl_id_[index], present_velocity[index]);
  }

  value[DXL_NUM-1] = gripper_controller.convertValue2Velocity(GRIPPER, present_velocity[DXL_NUM-1]);
}

bool OpenManipulatorMotorDriver::writeJointPosition(double *value)
{
  int32_t goal_position[JOINT_NUM] = {0, };

  for (int index = 0; index < JOINT_NUM; index++)
  {
    goal_position[index] = joint_controller.convertRadian2Value(dxl_id_[index], value[index]);
  }

  return joint_controller.syncWrite("Goal_Position", goal_position);
}

bool OpenManipulatorMotorDriver::writeGripperPosition(double value)
{
  return gripper_controller.itemWrite(GRIPPER, "Goal_Position", gripper_controller.convertRadian2Value(GRIPPER, value));
}