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

#include "turtlebot3_sensor.h"

Turtlebot3Sensor::Turtlebot3Sensor()
{
}

Turtlebot3Sensor::~Turtlebot3Sensor()
{  
}

bool Turtlebot3Sensor::init(void)
{
  battery_state_msg_.current         = NAN;
  battery_state_msg_.charge          = NAN;
  battery_state_msg_.capacity        = NAN;
  battery_state_msg_.design_capacity = NAN;
  battery_state_msg_.percentage      = NAN;

  imu_.begin();
}

void Turtlebot3Sensor::initIMU(void)
{
  imu_.begin();
}

void Turtlebot3Sensor::updateIMU(void)
{
  imu_.update();
}

void Turtlebot3Sensor::calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  const uint8_t led_ros_connect = 3;

  imu_.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!imu_.SEN.gyro_cali_get_done())
  {
    imu_.update();

    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      setLedToggle(led_ros_connect);
    }
  }
}

sensor_msgs::Imu Turtlebot3Sensor::getIMU(void)
{
  imu_msg_.angular_velocity.x = imu_.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg_.angular_velocity.y = imu_.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg_.angular_velocity.z = imu_.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.y = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.z = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  imu_msg_.orientation.w = imu_.quat[0];
  imu_msg_.orientation.x = imu_.quat[1];
  imu_msg_.orientation.y = imu_.quat[2];
  imu_msg_.orientation.z = imu_.quat[3];

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;
}

float* Turtlebot3Sensor::getOrientation(void)
{
  static float orientation[4];

  orientation[0] = imu_.quat[0];
  orientation[1] = imu_.quat[1];
  orientation[2] = imu_.quat[2];
  orientation[3] = imu_.quat[3];

  return orientation;
}

sensor_msgs::MagneticField Turtlebot3Sensor::getMag(void)
{
  mag_msg_.magnetic_field.x = imu_.SEN.magADC[0] * MAG_FACTOR;
  mag_msg_.magnetic_field.y = imu_.SEN.magADC[1] * MAG_FACTOR;
  mag_msg_.magnetic_field.z = imu_.SEN.magADC[2] * MAG_FACTOR;

  mag_msg_.magnetic_field_covariance[0] = 0.0048;
  mag_msg_.magnetic_field_covariance[1] = 0;
  mag_msg_.magnetic_field_covariance[2] = 0;
  mag_msg_.magnetic_field_covariance[3] = 0;
  mag_msg_.magnetic_field_covariance[4] = 0.0048;
  mag_msg_.magnetic_field_covariance[5] = 0;
  mag_msg_.magnetic_field_covariance[6] = 0;
  mag_msg_.magnetic_field_covariance[7] = 0;
  mag_msg_.magnetic_field_covariance[8] = 0.0048;

  return mag_msg_;
}

float Turtlebot3Sensor::checkVoltage(void)
{
  float vol_value;
  
  vol_value = getPowerInVoltage();

  return vol_value;
}

uint8_t Turtlebot3Sensor::checkPushButton(void)
{
  return getPushButton();
}