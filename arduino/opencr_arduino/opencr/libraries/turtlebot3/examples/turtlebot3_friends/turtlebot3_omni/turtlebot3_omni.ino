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

#include "turtlebot3_omni.h"

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

double linear_x                = 0.0;
double linear_y                = 0.0;
double angular_z               = 0.0;
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity   = 0.0;

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
  receiveRemoteControl();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlOmni);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Receive RC100 remote controller data
*******************************************************************************/
void receiveRemoteControl(void)
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
      linear_y -= VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }
    else if (received_data & RC100_BTN_R)
    {
      linear_y += VELOCITY_LINEAR_Y * SCALE_VELOCITY_LINEAR_Y;
    }

    if (received_data & RC100_BTN_1)
    {

    }
    else if (received_data & RC100_BTN_2)
    {
      angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_3)
    {

    }
    else if (received_data & RC100_BTN_4)
    {
      angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_6)
    {
      linear_x  = const_cmd_vel;
      linear_y  = 0.0;
      angular_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      linear_x  = 0.0;
      linear_y  = 0.0;
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

    goal_linear_x_velocity  = linear_x;
    goal_linear_y_velocity  = linear_y;
    goal_angular_velocity   = angular_z;
  }
}

/*******************************************************************************
* Control onmi speed
*******************************************************************************/
void controlOmni()
{
  bool dxl_comm_result = false;

  int64_t wheel_value[OMNIWHEEL_NUM] = {0, 0, 0};
  double wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (goal_linear_x_velocity * 0) + (goal_linear_y_velocity * (1 / WHEEL_RADIUS)) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[1] = (goal_linear_x_velocity * (sqrt(3) / (2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[2] = (goal_linear_x_velocity * (sqrt(3) / (-2 * WHEEL_RADIUS))) + (goal_linear_y_velocity * (-1 / (2 * WHEEL_RADIUS))) + (goal_angular_velocity * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));

  for (int id = 0; id < OMNIWHEEL_NUM; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * 9.54 /  RPM_CONSTANT_VALUE;

    if (wheel_value[id] > LIMIT_X_MAX_VALUE)       wheel_value[id] =  LIMIT_X_MAX_VALUE;
    else if (wheel_value[id] < -LIMIT_X_MAX_VALUE) wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

#ifdef DEBUG
  Serial.print("Vx : ");  Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);
#endif

  dxl_comm_result = motor_driver.controlMotor((int64_t)wheel_value[0], (int64_t)wheel_value[1], (int64_t)wheel_value[2]);
  if (dxl_comm_result == false)
    return;
}
