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

#ifndef OPEN_MANIPULATOR_DRIVER_H_
#define OPEN_MANIPULATOR_DRIVER_H_

#include <Dynamixel2Arduino.h>


enum JointMotorLocation{
  JOINT_1 = 0,
  JOINT_2,
  JOINT_3,
  JOINT_4,
  GRIPPER,
  JOINT_MOTOR_NUM_MAX
};

typedef struct
{
  int32_t value[JOINT_MOTOR_NUM_MAX];
} joint_position_info_t;

typedef struct
{
  int32_t value[JOINT_MOTOR_NUM_MAX];
} joint_velocity_info_t;

typedef struct
{
  int16_t value[JOINT_MOTOR_NUM_MAX];
} joint_current_info_t;

typedef struct
{
  uint32_t value[JOINT_MOTOR_NUM_MAX];
} joint_accel_info_t;


class OpenManipulatorDriver
{
 public:
  OpenManipulatorDriver(Dynamixel2Arduino &dxl_param);
  ~OpenManipulatorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();  

  bool set_torque(bool onoff);
  bool get_torque(void);
  
  bool read_goal_position(joint_position_info_t &position_info);
  bool read_present_position(joint_position_info_t &position_info);
  bool read_present_velocity(joint_velocity_info_t &velocity_info);
  bool read_present_current(joint_current_info_t &current_info);
  bool read_profile_acceleration(joint_accel_info_t &accel_info);  
  bool read_profile_velocity(joint_accel_info_t &accel_info);  
  bool read_goal_current(joint_current_info_t &current_info);
  
  bool write_goal_position_joint(joint_position_info_t &position_info);
  bool write_profile_acceleration_joint(joint_accel_info_t &accel_info);
  bool write_profile_velocity_joint(joint_accel_info_t &accel_info);
  bool write_goal_current_joint(joint_current_info_t &current_info);

  bool write_goal_position_gripper(joint_position_info_t &position_info);
  bool write_profile_acceleration_gripper(joint_accel_info_t &accel_info);
  bool write_profile_velocity_gripper(joint_accel_info_t &accel_info);
  bool write_goal_current_gripper(joint_current_info_t &current_info);

 private:
  
  bool is_ready(void);

  uint8_t motor_id_[JOINT_MOTOR_NUM_MAX];
  bool torque_;
  bool is_init_;
  bool is_connected_;
  Dynamixel2Arduino &dxl;
};

#endif // OPEN_MANIPULATOR_DRIVER_H_
