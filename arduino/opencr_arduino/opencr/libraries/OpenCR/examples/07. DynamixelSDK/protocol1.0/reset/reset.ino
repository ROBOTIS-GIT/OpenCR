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

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Factory Reset Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 1.0
// This example is tested with a Dynamixel MX-28, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

// Be aware that:
// This example resets all properties of Dynamixel to default values, such as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

#include <DynamixelSDK.h>

// Control table address
#define ADDR_MX_BAUDRATE                4                   // Control table address is different in Dynamixel model

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality

#define FACTORYRST_DEFAULTBAUDRATE      57600               // Dynamixel baudrate set by factoryreset
#define NEW_BAUDNUM                     1                   // New baudnum to recover Dynamixel baudrate as it was
#define OPERATION_MODE                  0x00                // Mode is unavailable in Protocol 1.0 Reset

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define CMD_SERIAL                      Serial

int getch()
{
  while(1)
  {
    if( CMD_SERIAL.available() > 0 )
    {
      break;
    }
  }

  return CMD_SERIAL.read();
}

int kbhit(void)
{
  return CMD_SERIAL.available();
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Start..");

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t dxl_baudnum_read;                       // Read baudnum

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    return;
  }

  // Read present baudrate of the controller
  Serial.print("Now the controller baudrate is :");
  Serial.print(portHandler->getBaudRate());

  // Try factoryreset
  Serial.print("[ID:"); Serial.print(DXL_ID);
  Serial.print("] Try factoryreset : ");

  dxl_comm_result = packetHandler->factoryReset(portHandler, DXL_ID, OPERATION_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print("Aborted\n");
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    return;
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Wait for reset
  Serial.print("Wait for reset...\n");
  delay(2000);

  Serial.print("[ID:"); Serial.print(DXL_ID);
  Serial.print("] factoryReset Success!\n");

  // Set controller baudrate to Dynamixel default baudrate
  if (portHandler->setBaudRate(FACTORYRST_DEFAULTBAUDRATE))
  {
    Serial.print("Succeed to change the controller baudrate to : ");
    Serial.print(FACTORYRST_DEFAULTBAUDRATE);
  }
  else
  {
    Serial.print("Failed to change the controller baudrate\n");
    return;
  }

  // Read Dynamixel baudnum
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_MX_BAUDRATE, &dxl_baudnum_read, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    Serial.print("[ID:"); Serial.print(DXL_ID);
    Serial.print("] DXL baudnum is now : ");
    Serial.println(dxl_baudnum_read);
  }

  // Write new baudnum
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_BAUDRATE, NEW_BAUDNUM, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    Serial.print("[ID:"); Serial.print(DXL_ID);
    Serial.print("] Set Dynamixel baudnum to : ");
    Serial.println(NEW_BAUDNUM);
  }

  // Set port baudrate to BAUDRATE
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeed to change the controller baudrate to : ");
    Serial.print(BAUDRATE);
  }
  else
  {
    Serial.print("Failed to change the controller baudrate\n");
    return;
  }

  delay(200);

  // Read Dynamixel baudnum
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_MX_BAUDRATE, &dxl_baudnum_read, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    Serial.print("[ID:"); Serial.print(DXL_ID);
    Serial.print("] Dynamixel Baudnum is now : ");
    Serial.print(dxl_baudnum_read);
  }

  // Close port
  portHandler->closePort();
}

void loop()
{
}
