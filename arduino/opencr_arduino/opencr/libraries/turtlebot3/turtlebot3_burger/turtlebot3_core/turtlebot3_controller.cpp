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

#include "turtlebot3_controller.h"

Turtlebot3Controller::Turtlebot3Controller()
{
  const_cmd_vel_ = CONST_VEL;
}

Turtlebot3Controller::~Turtlebot3Controller()
{  
}

bool Turtlebot3Controller::init(float max_lin_vel, float max_ang_vel, uint8_t scale_lin_vel, uint8_t scale_ang_vel)
{
  // 57600bps baudrate for RC100 control
  rc100_.begin(1);  

  max_lin_vel_ = max_lin_vel;
  max_ang_vel_ = max_ang_vel;
  scale_lin_vel_ = scale_lin_vel;
  scale_ang_vel_ = scale_ang_vel;
}

void Turtlebot3Controller::getRCdata(geometry_msgs::Twist *cmd_vel)
{
  uint8_t received_data = 0;
  
  if (rc100_.available())
  {
    received_data = rc100_.readData();

    if (received_data & RC100_BTN_U)
    {
      cmd_vel->linear.x += VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      cmd_vel->linear.x -= VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      cmd_vel->angular.z += VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      cmd_vel->angular.z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      cmd_vel->linear.x  = const_cmd_vel_;
      cmd_vel->angular.z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      cmd_vel->linear.x  = 0.0;
      cmd_vel->angular.z = 0.0;
    }
    else
    {
      cmd_vel->linear.x  = cmd_vel->linear.x;
      cmd_vel->angular.z = cmd_vel->angular.z;
    }

    if (cmd_vel->linear.x > max_lin_vel_)
    {
      cmd_vel->linear.x = max_lin_vel_;
    }

    if (cmd_vel->angular.z > max_ang_vel_)
    {
      cmd_vel->angular.z = max_ang_vel_;
    }
  }
}