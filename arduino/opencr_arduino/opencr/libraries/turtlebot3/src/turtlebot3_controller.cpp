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

#include "../include/turtlebot3_controller.h"

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

void Turtlebot3Controller::getRCdata(float *cmd_vel)
{
  uint16_t received_data = 0;

  static float lin_x = 0.0, ang_z = 0.0;
  
  if (rc100_.available())
  {
    received_data = rc100_.readData();

    if (received_data & RC100_BTN_U)
    {
      lin_x += VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      lin_x -= VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      ang_z += VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      ang_z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      lin_x  = const_cmd_vel_;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      lin_x  = 0.0;
      ang_z = 0.0;
    }
    else
    {
      lin_x  = lin_x;
      ang_z = ang_z;
    }

    if (lin_x > max_lin_vel_)
    {
      lin_x = max_lin_vel_;
    }

    if (ang_z > max_ang_vel_)
    {
      ang_z = max_ang_vel_;
    }

    cmd_vel[0] = lin_x;
    cmd_vel[1] = ang_z;
  }
}