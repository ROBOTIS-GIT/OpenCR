/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include <RC100.h>
#include <Dynamixel.h>

#define LEFT_DXL  1
#define RIGHT_DXL 2

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104

#define ON  1
#define OFF 0

#define VELOCITY 10

RC100 Controller;
int RcvData = 0;

Dynamixel Dxl;
int vel[4] = {LEFT_DXL, 0, RIGHT_DXL, 0};
int const_vel = 200;

unsigned int cur_time = 0;
unsigned int pre_time = 0;
bool experiment = false;
int dir = 0;

void setup() {
  Serial.begin(57600);

  Controller.begin(1);

  Dxl.begin(3);
  Dxl.writeByte(LEFT_DXL, ADDR_TORQUE_ENABLE, ON);
  Dxl.writeByte(RIGHT_DXL, ADDR_TORQUE_ENABLE, ON);
}

void loop() {
  if (Controller.available())
  {
    RcvData = Controller.readData();
    Serial.print("RcvData = ");
    Serial.print(RcvData);
    Serial.print(" LEFT_VEL = ");
    Serial.print(vel[1]);
    Serial.print(" RIGHT_VEL = ");
    Serial.println(vel[3]);

    if (RcvData & RC100_BTN_U)
    {
      vel[1] += VELOCITY;
      vel[3] -= VELOCITY;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_D)
    {
      vel[1] -= VELOCITY;
      vel[3] += VELOCITY;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_L)
    {
      vel[1] -= VELOCITY;
      vel[3] -= VELOCITY;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_R)
    {
      vel[1] += VELOCITY;
      vel[3] += VELOCITY;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_1)
    {
      const_vel += 10;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_3)
    {
      const_vel -= 10;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_2)
    {
      course();
    }
    else if (RcvData & RC100_BTN_6)
    {
      vel[1] = const_vel;
      vel[3] = -const_vel;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
    else if (RcvData & RC100_BTN_5)
    {
      vel[1] = 0;
      vel[3] = 0;

      Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
    }
  }
}

void course(void)
{
  vel[1] = 200;
  vel[3] = -200;

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);

  delay(9000);

  vel[1] = +100;
  vel[3] = +100;

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);

  delay(3300);

  vel[1] = 200;
  vel[3] = -200;

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);

  delay(9000);

  vel[1] = 0;
  vel[3] = 0;

  Dxl.syncWrite(ADDR_GOAL_VELOCITY, 1, vel, 4);
}

