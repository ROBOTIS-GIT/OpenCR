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

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <TurtleBot3_ROS2.h>
#include "turtlebot3_waffle.h"

#include <math.h>

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.0.0"

#define CONTROL_MOTOR_SPEED_FREQUENCY    30   //hz
 
#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define DEBUG                            
#define DEBUG_SERIAL                     SerialBT2
#define RTPS_SERIAL                      Serial
#ifdef DEBUG
  #define DEBUG_PRINT(x)                 DEBUG_SERIAL.print(x)
#else
  #define DEBUG_PRINT(x)                  
#endif


/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
Turtlebot3Controller controllers;
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};


/*******************************************************************************
* Declaration for DYNAMIXEL Slave Function
*******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const uint8_t ID_DXL_SLAVE = 200;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;

enum ControlTableItemAddr{
  ADDR_MODEL_NUMBER    = 0,
  ADDR_MODEL_INFORM    = 2,
  ADDR_FIRMWARE_VER    = 6,
  ADDR_ID              = 7,
  ADDR_BAUDRATE        = 8,
  
  ADDR_MILLIS          = 10,
  ADDR_MICROS          = 14,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,
  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,
  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,
  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,
  
  ADDR_PRESENT_CURRENT_L  = 120,
  ADDR_PRESENT_CURRENT_R  = 124,
  ADDR_PRESENT_VELOCITY_L = 128,
  ADDR_PRESENT_VELOCITY_R = 132,
  ADDR_PRESENT_POSITION_L = 136,
  ADDR_PRESENT_POSITION_R = 140,
  
  ADDR_MOTOR_TORQUE       = 149,
  ADDR_CMD_VEL_LINEAR_X   = 150,
  ADDR_CMD_VEL_LINEAR_Y   = 154,
  ADDR_CMD_VEL_LINEAR_Z   = 158,
  ADDR_CMD_VEL_ANGULAR_X  = 162,
  ADDR_CMD_VEL_ANGULAR_Y  = 166,
  ADDR_CMD_VEL_ANGULAR_Z  = 170,
  ADDR_PROFILE_ACC_L      = 174,
  ADDR_PROFILE_ACC_R      = 178
};

DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE, PROTOCOL_VERSION_DXL_SLAVE);

bool isAddrInRange(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length);
void dxl_slave_read_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length);
void dxl_slave_write_callback_func(DYNAMIXEL::Slave *slave, uint16_t addr, uint16_t length);

const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

#endif // TURTLEBOT3_CORE_CONFIG_H_

