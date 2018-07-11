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
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

// Be aware that:
// This example resets all properties of Dynamixel to default values, such as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#include <DynamixelSDK.h>                            // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_BAUDRATE               8                   // Control table address is different in Dynamixel model

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality

#define FACTORYRST_DEFAULTBAUDRATE      57600               // Dynamixel baudrate set by factoryreset
#define NEW_BAUDNUM                     3                   // New baudnum to recover Dynamixel baudrate as it was
#define OPERATION_MODE                  0x01                // 0xFF : reset all values
                                                            // 0x01 : reset all values except ID
                                                            // 0x02 : reset all values except ID and baudrate

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
  CMD_SERIAL.begin(115200);
  while(!CMD_SERIAL);

  CMD_SERIAL.println("Start..");

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
    CMD_SERIAL.print("Succeeded to open the port!\n");
  }
  else
  {
    CMD_SERIAL.print("Failed to open the port!\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    CMD_SERIAL.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    CMD_SERIAL.print("Failed to change the baudrate!\n");
    return;
  }

  // Read present baudrate of the controller
  CMD_SERIAL.print("Now the controller baudrate is :");
  CMD_SERIAL.print(portHandler->getBaudRate());

  // Try factoryreset
  CMD_SERIAL.print("[ID:"); CMD_SERIAL.print(DXL_ID);
  CMD_SERIAL.print("] Try factoryreset : ");

  dxl_comm_result = packetHandler->factoryReset(portHandler, DXL_ID, OPERATION_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    CMD_SERIAL.print("Aborted\n");
    CMD_SERIAL.print(packetHandler->getTxRxResult(dxl_comm_result));
    return;
  }
  else if (dxl_error != 0)
  {
    CMD_SERIAL.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Wait for reset
  CMD_SERIAL.print("Wait for reset...\n");
  delay(2000);

  CMD_SERIAL.print("[ID:"); CMD_SERIAL.print(DXL_ID);
  CMD_SERIAL.print("] factoryReset Success!\n");

  // Set controller baudrate to Dynamixel default baudrate
  if (portHandler->setBaudRate(FACTORYRST_DEFAULTBAUDRATE))
  {
    CMD_SERIAL.print("Succeed to change the controller baudrate to : ");
    CMD_SERIAL.println(FACTORYRST_DEFAULTBAUDRATE);
  }
  else
  {
    CMD_SERIAL.print("Failed to change the controller baudrate\n");
    return;
  }

  // Read Dynamixel baudnum
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUDRATE, &dxl_baudnum_read, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    CMD_SERIAL.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    CMD_SERIAL.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    CMD_SERIAL.print("[ID:"); CMD_SERIAL.print(DXL_ID);
    CMD_SERIAL.print("] DXL baudnum is now : ");
    CMD_SERIAL.println(dxl_baudnum_read);
  }

  // Write new baudnum
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUDRATE, NEW_BAUDNUM, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    CMD_SERIAL.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    CMD_SERIAL.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    CMD_SERIAL.print("[ID:"); CMD_SERIAL.print(DXL_ID);
    CMD_SERIAL.print("] Set Dynamixel baudnum to : ");
    CMD_SERIAL.println(NEW_BAUDNUM);
  }

  // Set port baudrate to BAUDRATE
  if (portHandler->setBaudRate(BAUDRATE))
  {
    CMD_SERIAL.print("Succeed to change the controller baudrate to : ");
    CMD_SERIAL.println(BAUDRATE);
  }
  else
  {
    CMD_SERIAL.print("Failed to change the controller baudrate\n");
    return;
  }

  delay(200);

  // Read Dynamixel baudnum
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUDRATE, &dxl_baudnum_read, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    CMD_SERIAL.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    CMD_SERIAL.print(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    CMD_SERIAL.print("[ID:"); CMD_SERIAL.print(DXL_ID);
    CMD_SERIAL.print("] Dynamixel Baudnum is now : ");
    CMD_SERIAL.print(dxl_baudnum_read);
  }

  // Close port
  portHandler->closePort();

  return;
}

void loop()
{
  
}