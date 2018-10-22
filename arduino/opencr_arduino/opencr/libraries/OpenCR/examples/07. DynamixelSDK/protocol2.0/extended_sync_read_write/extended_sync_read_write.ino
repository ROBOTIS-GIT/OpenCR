/*******************************************************************************
  Copyright (c) 2016, ROBOTIS CO., LTD.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

* * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

* * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

* * Neither the name of ROBOTIS nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Code started from sync_read_write example by: Ryu Woon Jung (Leon) */

// Updated by KurtE: to use two XL430-W250 running at 1MBS on
// OpenCM and OPenCR.
// This version was extended to have the SyncRead read in multiple addresses

// Also this version moves many of the main variables into global variables instead of being
// local to setup, as to avoid accidents of later creating global version, but not make
// the statements in Setup just be an assignement and then fault when you use them in loop...

//
// Available Dynamixel model on this example : All models using Protocol 2.0
//

#include <DynamixelSDK.h>

// Control table address
#define ADDR_X_TORQUE_ENABLE        64  // 1 0

#define ADDR_X_PROFILE_VELOCITY     112 // 4
#define ADDR_X_GOAL_POSITION        116 // 4
#define ADDR_X_PRESENT_LOAD         126 // 2 (R)
#define ADDR_X_PRESENT_VELOCITY     128 // 4 (R)
#define ADDR_X_PRESENT_POSITION     132 // 4 (R)


// Data Byte Length
#define LEN_X_PROFILE_VELOCITY        4
#define LEN_X_GOAL_POSITION           4
#define LEN_X_PRESENT_LOAD            2
#define LEN_X_PRESENT_VELOCITY        4
#define LEN_X_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        1000000
#if defined(__OPENCR__)
#define DEVICENAME                      "3"                 // This definition only has a symbolic meaning and does not affect to any functionality
#elif defined(__OPENCM904__)
#define DEVICENAME                      "3"                 // Open CM485 expansion board
//#define DEVICENAME                    "1"                 // Open CM905 built in ports
#endif

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE     1024                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE     3072                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_VELOCITY_1                  100                 // First moves speed
#define DXL_VELOCITY_2                  300                 // Second moves speed
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define DXL_MOVING_VELOCITY_THRESHOLD   3                   // Not sure what is good one here

#define ESC_ASCII_VALUE                 0x1b


//----------------------------------------------------------------------
// Globals
//----------------------------------------------------------------------
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

#define COUNT_SERVOS 2

const uint8_t servo_list[COUNT_SERVOS] = {DXL1_ID, DXL2_ID};

// Question to self and others.  Ok to have global GroupSync objects?  Nope...
// Initialize GroupSyncWrite instance
#define GLOBAL_SYNC_OBJECTS

#ifdef GLOBAL_SYNC_OBJECTS
dynamixel::GroupSyncWrite gsw(ADDR_X_PROFILE_VELOCITY, LEN_X_PROFILE_VELOCITY + LEN_X_GOAL_POSITION);
uint8_t gsw_buffer[(LEN_X_PROFILE_VELOCITY + LEN_X_GOAL_POSITION + dynamixel::GroupSyncWrite::EXTRA_BYTES_PER_ITEM) * COUNT_SERVOS];

dynamixel::GroupSyncRead gsr(ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD + LEN_X_PRESENT_VELOCITY + LEN_X_PRESENT_POSITION);
uint8_t gsr_buffer[(LEN_X_PRESENT_LOAD + LEN_X_PRESENT_VELOCITY + LEN_X_PRESENT_POSITION + dynamixel::GroupSyncRead::EXTRA_BYTES_PER_ITEM) * COUNT_SERVOS];
#endif

dynamixel::GroupSyncWrite *groupSyncWrite;

// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead *groupSyncRead;

const uint32_t dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
const uint32_t dxl_goal_velocity[2] = {DXL_VELOCITY_1, DXL_VELOCITY_2};
int goal_position_index = 0;

//----------------------------------------------------------------------
// Forward references
//----------------------------------------------------------------------
extern bool report_any_dxl_errors(int dxl_comm_result, uint8_t dxl_error, uint8_t id);

//----------------------------------------------------------------------

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);   // enable the LED pin

  Serial.begin(115200);
  while (!Serial);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  //bool dxl_getdata_result = false;                 // GetParam result

  uint8_t dxl_error = 0;                          // Dynamixel error

  // Initialize PortHandler instance
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

#ifdef GLOBAL_SYNC_OBJECTS
  gsw.init(portHandler, packetHandler);
  gsw.setBuffer(gsw_buffer, sizeof(gsw_buffer));
  groupSyncWrite = &gsw;

  gsr.init(portHandler, packetHandler);
  gsr.setBuffer(gsr_buffer, sizeof(gsr_buffer));
  groupSyncRead = &gsr;
#else
  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD + LEN_X_PRESENT_VELOCITY + LEN_X_PRESENT_POSITION);
#endif

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
  Serial.println("Before Torque Enable"); Serial.flush();

  // Now loop through and initialize each of the servos.
  for (uint8_t servo_index = 0; servo_index < COUNT_SERVOS; servo_index++)
  {
    // enable torque on the servo
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_list[servo_index], ADDR_X_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (!report_any_dxl_errors(dxl_comm_result, dxl_error, servo_list[servo_index]))
    {
      Serial.print("[ID:"); Serial.print(servo_list[servo_index]); Serial.println("] Torque enabled");
    }

    // Add the servo to the group read list
    if (!groupSyncRead->addParam(servo_list[servo_index]))
    {
      Serial.printf("groupSyncRead addparam failed Dynamixel id[%d]\n", servo_list[servo_index]);
    }
  }
}

void loop()
{
  int32_t dxl_present_position[COUNT_SERVOS];      // Present position
  int32_t dxl_present_velocity[COUNT_SERVOS];      // Present velocity
  int32_t dxl_present_load[COUNT_SERVOS];          // Present load
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool still_moving;

  Serial.println("Press any key to continue! (or press q to quit!)\n");

  while ( Serial.available() == 0 )  ;

  uint8_t ch = Serial.read(); // get the first char
  if (ch == 'q')
  {
    quit_test();
  }
  while (Serial.available())
  {
    Serial.read();  // clear out any other chars
  }

  // Now loop through and initialize each of the servos.
  for (uint8_t servo_index = 0; servo_index < COUNT_SERVOS; servo_index++)
  {
    // Add Dynamixels goal position value to the Syncwrite storage
    if (!groupSyncWrite->setParam(servo_list[servo_index], ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION,
                                  dxl_goal_position[goal_position_index]))
    {
      Serial.printf("groupSyncWrite setParam goal position failed Dynamixel id[%d]\n", servo_list[servo_index]);
    }

    // Add Dynamixels profile velocity
    if (!groupSyncWrite->setParam(servo_list[servo_index], ADDR_X_PROFILE_VELOCITY, LEN_X_PROFILE_VELOCITY,
                                  dxl_goal_velocity[goal_position_index]))
    {
      Serial.printf("groupSyncWrite setParam velocity failed Dynamixel id[%d]\n", servo_list[servo_index]);
    }
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite->txPacket();
  report_any_dxl_errors(dxl_comm_result, 0, 0xfe);

  // Clear syncwrite parameter storage - now simply sets count to zero
  groupSyncWrite->clearParam();

  do
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // play with LED pin
    // Syncread present position
    dxl_comm_result = groupSyncRead->txRxPacket();
    report_any_dxl_errors(dxl_comm_result, 0, 0xfe);

    still_moving = false; // See if any of the servos still show as moving.

    for (uint8_t servo_index = 0; servo_index < COUNT_SERVOS; servo_index++)
    {
      // Check for any character input, if so break out...
      if (Serial.available())
      {
        while (Serial.available())
        {
          Serial.read();  // remove all before continue
        }
        break;
      }
      // Check if groupsyncread data of Dynamixel#1 is available
      if (groupSyncRead->isAvailable(servo_list[servo_index], ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION))
      {
        // Get Dynamixel#1 present values
        dxl_present_position[servo_index] = groupSyncRead->getData(servo_list[servo_index], ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
        dxl_present_velocity[servo_index] = groupSyncRead->getData(servo_list[servo_index], ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
        dxl_present_load[servo_index] =     groupSyncRead->getData(servo_list[servo_index], ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD);

        Serial.print("[ID:"); Serial.print(servo_list[servo_index]);
        Serial.print("] Goal:"); Serial.print(dxl_goal_position[goal_position_index]);
        Serial.print(" "); Serial.print(dxl_goal_velocity[goal_position_index]);
        Serial.print(" Pres(p/v/l):"); Serial.print(dxl_present_position[servo_index]);
        Serial.print(" "); Serial.print(dxl_present_velocity[servo_index]);
        Serial.print(" "); Serial.print(dxl_present_load[servo_index]);

        if ((abs((int)dxl_goal_position[goal_position_index] - dxl_present_position[servo_index]) > DXL_MOVING_STATUS_THRESHOLD) 
          || (abs(dxl_present_velocity[servo_index]) > DXL_MOVING_VELOCITY_THRESHOLD)) 
        {
          still_moving = true;
        }
      }
      else
      {
        Serial.print("[ID:"); Serial.print(servo_list[servo_index]); Serial.print("] groupSyncRead getdata failed");
      }
    }
    Serial.println(); // end the current line

  } while (still_moving) ;

  // Change goal position
  if (goal_position_index == 0)
  {
    goal_position_index = 1;
  }
  else
  {
    goal_position_index = 0;
  }
}



void quit_test ()
{
  int dxl_comm_result;
  uint8_t dxl_error;

  Serial.println("\n\n***** Test ending ***");

  // Disable Dynamixel#1 Torque
  // Now loop through and initialize each of the servos.
  for (uint8_t servo_index = 0; servo_index < COUNT_SERVOS; servo_index++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo_list[servo_index], ADDR_X_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    report_any_dxl_errors(dxl_comm_result, dxl_error, servo_list[servo_index]);
  }

  // Close port
  portHandler->closePort();

  Serial.println("Test completed, must reset board to do anything...");
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // play with LED pin
    delay(500);
  }
}


bool report_any_dxl_errors(int dxl_comm_result, uint8_t dxl_error, uint8_t id)
{
  if (dxl_comm_result != COMM_SUCCESS)
  {
    if (id != 0xfe)
    {
      Serial.print("ID["); Serial.print(id, DEC); Serial.print("] ");
    }
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    return true;
  }
  else if (dxl_error != 0)
  {
    if (id != 0xfe)
    {
      Serial.print("ID["); Serial.print(id, DEC); Serial.print("] ");
    }
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    return true;
  }

  return false; // did not report anything
}
