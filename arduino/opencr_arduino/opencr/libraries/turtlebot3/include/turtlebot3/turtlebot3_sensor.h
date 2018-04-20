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

#ifndef TURTLEBOT3_SENSOR_H_
#define TURTLEBOT3_SENSOR_H_

#include <IMU.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>

#define ACCEL_FACTOR                     -0.000598  // 2.0 * -9.8 / 32768    adc * 2/32768 = g
#define GYRO_FACTOR                       0.000133  // pi / (131 * 180)      adc * 1/16.4 = deg/s => 1/16.4 deg/s -> 0.001064225157909 rad/s
#define MAG_FACTOR                       6e-7

class Turtlebot3Sensor
{
 public:
  Turtlebot3Sensor();
  ~Turtlebot3Sensor();

  bool init(void);

  void initIMU(void);
  sensor_msgs::Imu getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float* getOrientation(void);

  sensor_msgs::MagneticField getMag(void);

  float checkVoltage(void);

  uint8_t checkPushButton(void);  

 private:
  sensor_msgs::Imu           imu_msg_;
  sensor_msgs::BatteryState  battery_state_msg_;
  sensor_msgs::MagneticField mag_msg_;

  cIMU imu_;
};

#endif // TURTLEBOT3_SENSOR_H_
