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

/* Author: Taehoon Lim (Darby), Ashe Kim */

#include <RC100.h>
#include <DynamixelSDK.h>

#define LEFT_DXL  1
#define RIGHT_DXL 2

#define BAUDRATE 1000000
#define DEVICENAME ""
#define PROTOCOL_VERSION 2.0

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104

#define LEN_TORQUE_ENABLE 1
#define LEN_GOAL_VELOCITY 4

#define ON  1
#define OFF 0

#define VELOCITY 10

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

dynamixel::GroupSyncWrite *groupSyncWrite;

bool dxl_addparam_result = false;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

int vel[2] = {0, 0};
int const_vel = 200;

RC100 Controller;
int RcvData = 0;

void setup() 
{
  Serial.begin(57600);

  Controller.begin(1);

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

  portHandler -> openPort();
  portHandler->setBaudRate(BAUDRATE);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, LEFT_DXL, ADDR_TORQUE_ENABLE, ON, &dxl_error);
  packetHandler->getTxRxResult(dxl_comm_result);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, RIGHT_DXL, ADDR_TORQUE_ENABLE, ON, &dxl_error);
  packetHandler->getTxRxResult(dxl_comm_result);
}

void loop() 
{
  if (Controller.available())
  {
    RcvData = Controller.readData();
    Serial.print("RcvData = ");
    Serial.print(RcvData);
    Serial.print(" LEFT_VEL = ");
    Serial.print(vel[0]);
    Serial.print(" RIGHT_VEL = ");
    Serial.println(vel[1]);

    if (RcvData & RC100_BTN_U)
    {
      vel[0] += VELOCITY;
      vel[1] -= VELOCITY;
    }
    else if (RcvData & RC100_BTN_D)
    {
      vel[0] -= VELOCITY;
      vel[1] += VELOCITY;
    }
    else if (RcvData & RC100_BTN_L)
    {
      vel[0] -= VELOCITY;
      vel[1] -= VELOCITY;
    }
    else if (RcvData & RC100_BTN_R)
    {
      vel[0] += VELOCITY;
      vel[1] += VELOCITY;
    }
    else if (RcvData & RC100_BTN_1)
    {
      vel[0] = (const_vel + 10);
      vel[1] = (const_vel + 10);
    }
    else if (RcvData & RC100_BTN_2)
    {
      vel[0] = (const_vel - 10);
      vel[1] = (const_vel - 10);
    }
    else if (RcvData & RC100_BTN_3)
    {
      vel[0] = -const_vel;
      vel[1] = const_vel;
    }
    else if (RcvData & RC100_BTN_4)
    {
      vel[0] = const_vel;
      vel[1] = -const_vel;
    }
    else if (RcvData & RC100_BTN_5)
    {
      vel[0] = 0;
      vel[1] = 0;
    }

    controlMotor( vel[0], vel[1]);
    return;
  }
}

void controlMotor( int64_t left_wheel_value, int64_t right_wheel_value)
{
  //bool dxl_addparam_result;
  //int dxl_comm_result;

  groupSyncWrite->addParam(LEFT_DXL, (uint8_t*)&left_wheel_value);
  groupSyncWrite->addParam(RIGHT_DXL, (uint8_t*)&right_wheel_value);

  groupSyncWrite->txPacket();

  groupSyncWrite->clearParam();
}
