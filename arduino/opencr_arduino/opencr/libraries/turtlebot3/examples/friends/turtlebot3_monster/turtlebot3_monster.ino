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

#include "turtlebot3_monster.h"

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
  Timer.attachInterrupt(controlMonster);
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
* Control bike speed
*******************************************************************************/
void controlMonster()
{
  bool dxl_comm_result = false;

  double wheel1_spd_cmd, wheel2_spd_cmd, wheel3_spd_cmd, wheel4_spd_cmd;
  double lin_vel1, lin_vel2, lin_vel3, lin_vel4;

  double rotation_center;

  wheel1_spd_cmd = goal_linear_velocity - (sqrt(WHEEL_POS_FROM_CENTER_X_1 * WHEEL_POS_FROM_CENTER_X_1 + WHEEL_POS_FROM_CENTER_Y_1 * WHEEL_POS_FROM_CENTER_Y_1) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_1 / WHEEL_POS_FROM_CENTER_X_1));

  wheel2_spd_cmd = goal_linear_velocity + (sqrt(WHEEL_POS_FROM_CENTER_X_2 * WHEEL_POS_FROM_CENTER_X_2 + WHEEL_POS_FROM_CENTER_Y_2 * WHEEL_POS_FROM_CENTER_Y_2) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_2 / WHEEL_POS_FROM_CENTER_X_2));

  wheel3_spd_cmd = goal_linear_velocity - (sqrt(WHEEL_POS_FROM_CENTER_X_3 * WHEEL_POS_FROM_CENTER_X_3 + WHEEL_POS_FROM_CENTER_Y_3 * WHEEL_POS_FROM_CENTER_Y_3) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_3 / WHEEL_POS_FROM_CENTER_X_3));

  wheel4_spd_cmd = goal_linear_velocity + (sqrt(WHEEL_POS_FROM_CENTER_X_4 * WHEEL_POS_FROM_CENTER_X_4 + WHEEL_POS_FROM_CENTER_Y_4 * WHEEL_POS_FROM_CENTER_Y_4) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_4 / WHEEL_POS_FROM_CENTER_X_4));

  lin_vel1 = wheel1_spd_cmd * VELOCITY_CONSTANT_VAULE;
  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel2 = -1 * wheel2_spd_cmd * VELOCITY_CONSTANT_VAULE;
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel3 = wheel3_spd_cmd * VELOCITY_CONSTANT_VAULE;
  if (lin_vel3 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel3 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel3 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel4 = -1 * wheel4_spd_cmd * VELOCITY_CONSTANT_VAULE;
  if (lin_vel4 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel4 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel4 = -LIMIT_X_MAX_VELOCITY;
  }

  dxl_comm_result = motor_driver.controlMotor((int64_t)lin_vel1, (int64_t)lin_vel2, (int64_t)lin_vel3, (int64_t)lin_vel4);
  if (dxl_comm_result == false)
    return;
}
