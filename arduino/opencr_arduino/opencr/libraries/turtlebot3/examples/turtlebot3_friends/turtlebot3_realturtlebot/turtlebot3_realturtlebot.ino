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

#include "turtlebot3_realturtlebot.h"
#include "turtlebot3_realturtlebot_motion.h"

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Declaration for RC100 remote conroller
*******************************************************************************/
RC100 remote_controller;

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

TurtlebotMotion turtlebotMotion;

void setup()
{
  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for RC100 remote control and cmd_vel
  remote_controller.begin(1);  //57600bps for RC100

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  // Init Motion
  controlRealTurtleBot();
}

void loop()
{
  receiveRemoteControlData();
}

void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlRealTurtleBot);
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

    turtlebotMotion.getDirection(received_data);

    controlRealTurtleBot();

    remote_controller.begin(1);  // refresh remote controller buffer
  }
}

/*******************************************************************************
* Control Real TurtleBot
*******************************************************************************/
void controlRealTurtleBot()
{
  int pres_pos[8] = {0, };
  int* goal_pos;
  double dist[8] = {0.0, };
  int goal_pos_dist_int[8] = {0, };
  double goal_pos_dist_db[8] = {0.0, };

  turtlebotMotion.setParams();

  for (int motion_num = 0; motion_num < turtlebotMotion.motion_all_num; motion_num++)
  {
    double dist_number = turtlebotMotion.time_duration[motion_num] / 0.008;

    goal_pos = turtlebotMotion.setJointAngle(motion_num);
    motor_driver.syncRead(ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION, pres_pos);

    for (int joint_num = 0; joint_num < 8; joint_num++)
    {
      dist[joint_num] = (goal_pos[joint_num] * 1.0 - pres_pos[joint_num] * 1.0) / dist_number;
      goal_pos_dist_db[joint_num] = pres_pos[joint_num] * 1.0;
    }

    for (int res_num = 0; res_num < (int)dist_number; res_num++)
    {
      for (int joint_num = 0; joint_num < 8; joint_num++)
      {
        goal_pos_dist_db[joint_num] += dist[joint_num];
        goal_pos_dist_int[joint_num] = (int)goal_pos_dist_db[joint_num];
      }

      motor_driver.syncWrite(ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, goal_pos_dist_int);

      delay(8);
    }

    delay(turtlebotMotion.time_space[motion_num] * 1000.0);
  }
}
