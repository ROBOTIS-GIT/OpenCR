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
// *********     Indirect Address Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#include <DynamixelSDK.h>


// Control table address
// Control table address is different in Dynamixel model
#define ADDR_PRO_INDIRECTADDRESS_FOR_WRITE      49                  // EEPROM region
#define ADDR_PRO_INDIRECTADDRESS_FOR_READ       59                  // EEPROM region
#define ADDR_PRO_TORQUE_ENABLE                  562
#define ADDR_PRO_LED_RED                        563
#define ADDR_PRO_GOAL_POSITION                  596
#define ADDR_PRO_MOVING                         610
#define ADDR_PRO_PRESENT_POSITION               611
#define ADDR_PRO_INDIRECTDATA_FOR_WRITE         634
#define ADDR_PRO_INDIRECTDATA_FOR_READ          639

// Data Byte Length
#define LEN_PRO_LED_RED                         1
#define LEN_PRO_GOAL_POSITION                   4
#define LEN_PRO_MOVING                          1
#define LEN_PRO_PRESENT_POSITION                4
#define LEN_PRO_INDIRECTDATA_FOR_WRITE          5
#define LEN_PRO_INDIRECTDATA_FOR_READ           5

// Protocol version
#define PROTOCOL_VERSION                        2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                                  1                   // Dynamixel ID: 1
#define BAUDRATE                                57600
#define DEVICENAME                              "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality

#define TORQUE_ENABLE                           1                   // Value for enabling the torque
#define TORQUE_DISABLE                          0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE             -150000              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE              150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_LED_VALUE                   0                   // Dynamixel LED will light between this value
#define DXL_MAXIMUM_LED_VALUE                   255                 // and this value
#define DXL_MOVING_STATUS_THRESHOLD             20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                         0x1b

#define CMD_SERIAL                              Serial

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

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE, LEN_PRO_INDIRECTDATA_FOR_WRITE);

  // Initialize Groupsyncread instance
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_INDIRECTDATA_FOR_READ);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;               // addParam result
  bool dxl_getdata_result = false;                // GetParam result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};  // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t dxl_moving = 0;                         // Dynamixel moving status
  uint8_t param_indirect_data_for_write[LEN_PRO_INDIRECTDATA_FOR_WRITE];
  uint8_t dxl_led_value[2] = {0x00, 0xFF};        // Dynamixel LED value
  int32_t dxl_present_position = 0;               // Present position

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

  // Disable Dynamixel Torque :
  // Indirect address would not accessible when the torque is already enabled
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
    Serial.print("Dynamixel has been successfully connected \n");
  }

  // INDIRECTDATA parameter storages replace LED, goal position, present position and moving status storages
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_LED_RED, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_INDIRECTADDRESS_FOR_READ + 8, ADDR_PRO_MOVING, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Add parameter storage for the present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL_ID);
  if (dxl_addparam_result != true)
  {
    Serial.print("[ID:"); Serial.print(DXL_ID); Serial.println("groupSyncRead addparam failed");
    return;
  }

  while(1)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");
    if (getch() == 'q')
      break;

    // Allocate LED and goal position value into byte array
    param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_indirect_data_for_write[4] = dxl_led_value[index];

    // Add values to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_indirect_data_for_write);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DXL_ID); Serial.println("groupSyncWrite addparam failed");
      return;
    }

    // Syncwrite all
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Syncread present position from indirectdata2
      dxl_comm_result = groupSyncRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));

      // Check if groupsyncread data of Dyanamixel is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL_ID); Serial.println("groupSyncRead getdata failed");
        return;
      }

      // Check if groupsyncread data of Dyanamixel is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID, ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
      if (dxl_getdata_result != true)
      {
        Serial.print("[ID:"); Serial.print(DXL_ID); Serial.println("groupSyncRead getdata failed");
        return;
      }

      // Get Dynamixel present position value
      dxl_present_position = groupSyncRead.getData(DXL_ID, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);

      // Get Dynamixel moving status value
      dxl_moving = groupSyncRead.getData(DXL_ID, ADDR_PRO_INDIRECTDATA_FOR_READ + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);

      Serial.print("[ID:"); Serial.print(DXL_ID);
      Serial.print("] GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print("  PresPos:"); Serial.print(dxl_present_position);
      Serial.print("  IsMoving:"); Serial.print(dxl_moving);
      Serial.println(" ");

    }while(abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD);

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return;
}

void loop()
{
  
}