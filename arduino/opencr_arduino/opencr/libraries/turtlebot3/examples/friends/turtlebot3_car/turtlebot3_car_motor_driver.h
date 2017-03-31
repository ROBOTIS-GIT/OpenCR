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

#ifndef TURTLEBOT3_BIKE_MOTOR_DRIVER_H_
#define TURTLEBOT3_BIKE_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_REAR_ID                     2       // ID of rear motor
#define DXL_FRONT_ID                    1       // ID of front motor
#define BAUDRATE                        1000000 // baud rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define TURN_VALUE                      204

#define X_POS_MIN                       0
#define X_POS_MAX                       4095
#define X_POS_CENTER                    2868 // Don't need to use this value... actual center position value of the Dynamixel is 2048

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);
  bool controlMotor(int64_t front_joint_value, int64_t rear_wheel_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t rear_wheel_id_;
  uint8_t front_joint_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupBulkWrite *groupBulkWrite_;
};

#endif // TURTLEBOT3_CAR_MOTOR_DRIVER_H_
