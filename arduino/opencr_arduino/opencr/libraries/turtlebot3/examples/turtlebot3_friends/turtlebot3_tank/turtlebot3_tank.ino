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

#include "turtlebot3_tank.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel    = 0.2;

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

double linear_x              = 0.0;
double angular_z             = 0.0;
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  remote_controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  // Start Dynamixel Control Interrupt
  startDynamixelControlInterrupt();
}

void loop()
{
  receiveRemoteControlData();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlTank);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControlData(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if (received_data & RC100_BTN_U)
    {
      linear_x  += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if (received_data & RC100_BTN_D)
    {
      linear_x  -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }

    if (received_data & RC100_BTN_L)
    {
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_R)
    {
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_1)
    {

    }
    else if (received_data & RC100_BTN_2)
    {

    }
    else if (received_data & RC100_BTN_3)
    {

    }
    else if (received_data & RC100_BTN_4)
    {

    }

    if (received_data & RC100_BTN_6)
    {
      linear_x  = const_cmd_vel;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      linear_x  = 0.0;
      angular_z = 0.0;
    }

    if (linear_x > MAX_LINEAR_VELOCITY)
    {
      linear_x = MAX_LINEAR_VELOCITY;
    }

    if (angular_z > MAX_ANGULAR_VELOCITY)
    {
      angular_z = MAX_ANGULAR_VELOCITY;
    }

    goal_linear_velocity  = linear_x;
    goal_angular_velocity = angular_z;
  }
}

/*******************************************************************************
* Control tank speed
*******************************************************************************/
void controlTank()
{
  bool dxl_comm_result = false;

  const int8_t motor_num = 2;

  int64_t wheel_value[motor_num] = {0, 0};             //LEFT, RIGHT
  double wheel_angular_velocity[motor_num] = {0.0, 0.0};

  wheel_angular_velocity[0] = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);
  wheel_angular_velocity[1] = goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2);

  for (int id = 0; id < motor_num; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * VELOCITY_CONSTANT_VAULE;

    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

  dxl_comm_result = motor_driver.controlMotor((int64_t)wheel_value[0], (int64_t)wheel_value[1]);
  if (dxl_comm_result == false)
    return;
}
