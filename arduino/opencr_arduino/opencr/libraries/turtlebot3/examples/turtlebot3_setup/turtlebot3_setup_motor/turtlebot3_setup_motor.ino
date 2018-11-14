/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
#include <DynamixelSDK.h>
#include <stdarg.h>

// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#if defined(__OPENCR__) 
#define DEVICENAME                      "/dev/OpenCR"       // Device name not used on OpenCR
#elif defined(__OPENCM904__)
#define DEVICENAME                      "3"                 // Default to external (OpenCM485 expansion) on OpenCM9.04
#endif

// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define CMD_SERIAL                      Serial              // USB Serial



typedef union
{
  uint8_t  u8Data[4];
  uint16_t u16Data[2];
  uint32_t u32Data;

  int8_t   s8Data[4];
  int16_t  s16Data[2];
  int32_t  s32Data;
} dxl_ret_t;

char *dev_name = (char*)DEVICENAME;

// Initialize Packethandler2 instance
dynamixel::PacketHandler *packetHandler2;
dynamixel::PortHandler   *portHandler;


int tb3_id = -1;
int tb3_baud = -1;


bool requestConfirm(void);
bool findMotor(int id);
bool setupMotorLeft(void);
bool setupMotorRight(void);
void testMotor(uint8_t id);


void      write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value);
dxl_ret_t read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length);



void setup()
{
  CMD_SERIAL.begin(57600);
  while (!CMD_SERIAL);


  packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
  portHandler    = dynamixel::PortHandler::getPortHandler(dev_name);

  // Open port
  if (portHandler->openPort())
  {
    CMD_SERIAL.println("Succeeded to open the port!");
    CMD_SERIAL.printf(" - Device Name : %s\r\n", dev_name);
    CMD_SERIAL.printf(" - Baudrate    : %d\r\n", portHandler->getBaudRate());
    tb3_baud = portHandler->getBaudRate();
  }
  else
  {
    CMD_SERIAL.printf("Failed to open the port! [%s]\n", dev_name);
    CMD_SERIAL.printf("Press any key to terminate...\n");
    while (1);
  }


  CMD_SERIAL.println("\r\nStart turtlebot3 setup motor");
  drawTitle();
}

void loop()
{
  uint8_t ch;

  if (CMD_SERIAL.available())
  {
    ch = CMD_SERIAL.read();

    if (ch == '1')
    {
      flushCmd();
      if (requestConfirm() == true)
      {
        CMD_SERIAL.println("setup.... left");

        if (findMotor(1) == true)
        {
          setupMotorLeft();
        }
      }
    }
    if (ch == '2')
    {
      flushCmd();
      if (requestConfirm() == true)
      {
        CMD_SERIAL.println("setup.... right");

        if (findMotor(2) == true)
        {
          setupMotorRight();
        }
      }
    }
    if (ch == '3')
    {
      flushCmd();
      CMD_SERIAL.println("test.... left");
      testMotor(1);
    }
    if (ch == '4')
    {
      flushCmd();
      CMD_SERIAL.println("test.... right");
      testMotor(2);
    }

    drawTitle();
    flushCmd();
  }
}

void drawTitle(void)
{
  CMD_SERIAL.println(" ");
  CMD_SERIAL.println(" ");
  CMD_SERIAL.println("1. setup left  motor");
  CMD_SERIAL.println("2. setup right motor");
  CMD_SERIAL.println("3. test  left  motor");
  CMD_SERIAL.println("4. test  right motor");
  CMD_SERIAL.print(">> ");
}

bool requestConfirm(void)
{
  uint8_t ch;


  CMD_SERIAL.print("Do you really want to setup ? y/n : ");

  while (1)
  {
    if (CMD_SERIAL.available())
    {
      ch = CMD_SERIAL.read();

      if (ch == 'y' || ch == 'Y')
      {
        CMD_SERIAL.println("yes");
        flushCmd();
        return true;
      }

      break;
    }
  }

  CMD_SERIAL.println("no");
  return false;
}

void flushCmd(void)
{
  uint8_t ch;

  while (CMD_SERIAL.available())
  {
    ch = CMD_SERIAL.read();
  }
}


bool findMotor(int id)
{
  uint32_t baud_tbl[2] = { 57600, 1000000 };
#define COUNT_BAUD (sizeof(baud_tbl)/sizeof(baud_tbl[0]))
  uint32_t index;
  uint32_t baud_pre;

  std::vector<unsigned char> vec;


  baud_pre = portHandler->getBaudRate();

  tb3_id = -1;

  CMD_SERIAL.println("Find Motor...");

  // First try to find the specific servo ID wanted
  for (index = 0; index < COUNT_BAUD; index++)
  {
    portHandler->setBaudRate(baud_tbl[index]);
    uint16_t model_number;
    int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      if (tb3_id == -1)
      {
        tb3_id = id;
        tb3_baud =  baud_tbl[index];
      }
      else
      {
        CMD_SERIAL.printf("Warning Servo %d found at two baud rates %d and %d using %d\n",
                          id, tb3_baud, baud_tbl[index], baud_tbl[index]);
        tb3_baud =  baud_tbl[index];
      }
    }
  }

  if (tb3_id != -1)
  {
    CMD_SERIAL.println("    ... SUCCESS");
    CMD_SERIAL.printf("    [ID: %d found at baud: %d]\n", id, tb3_baud );
    portHandler->setBaudRate(tb3_baud);
    return true;
  }

  // Did not find the actual ID we were looking for so see if we find any servos?
  for (index = 0; index < COUNT_BAUD; index++)
  {
    CMD_SERIAL.printf("    setbaud : %d\r\n", baud_tbl[index]);

    portHandler->setBaudRate(baud_tbl[index]);
    tb3_baud =  baud_tbl[index];

    // Lets see if we find the actual one we want to update?
    int dxl_comm_result = packetHandler2->broadcastPing(portHandler, vec);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      CMD_SERIAL.println(packetHandler2->getTxRxResult(dxl_comm_result));
      continue;
    }

    for (unsigned int i = 0; i < vec.size(); i++)
    {
      CMD_SERIAL.println("    ... SUCCESS");
      CMD_SERIAL.println("    [ID:" + String(vec.at(i)) + "]");
      tb3_id = vec.at(i);
    }

    if (vec.size() > 0)
    {
      CMD_SERIAL.println("    found motor");
      if (vec.size() > 1)
      {
        CMD_SERIAL.printf("    WARNING: multiple servos found, using id: %d\n", tb3_id);
        if (!requestConfirm())
        {
          tb3_id = -1;  // setup to abort...
        }
      }

      break;
    }
    else
    {
      CMD_SERIAL.println("    not found");
    }
  }

  if (tb3_id < 0)
  {
    portHandler->setBaudRate(baud_pre);
    return false;
  }
  else
  {
    return true;
  }
}

bool setupMotorLeft(void)
{
  CMD_SERIAL.println("Setup Motor Left...");


  if (tb3_id < 0)
  {
    CMD_SERIAL.println("    no dxl motors");
  }
  else
  {
    write(portHandler, packetHandler2, tb3_id, 64, 1, 0);
    write(portHandler, packetHandler2, tb3_id, 7, 1, 1);
    tb3_id = 1;
    write(portHandler, packetHandler2, tb3_id, 8, 1, 3);
    portHandler->setBaudRate(1000000);
    write(portHandler, packetHandler2, tb3_id, 10, 1, 0);
    write(portHandler, packetHandler2, tb3_id, 11, 1, 1);
    CMD_SERIAL.println("    ok");
  }
}

bool setupMotorRight(void)
{
  CMD_SERIAL.println("Setup Motor Right...");


  if (tb3_id < 0)
  {
    CMD_SERIAL.println("    no dxl motors");
  }
  else
  {
    write(portHandler, packetHandler2, tb3_id, 64, 1, 0);
    write(portHandler, packetHandler2, tb3_id, 7, 1, 2);
    tb3_id = 2;
    write(portHandler, packetHandler2, tb3_id, 8, 1, 3);
    portHandler->setBaudRate(1000000);
    write(portHandler, packetHandler2, tb3_id, 10, 1, 1);
    write(portHandler, packetHandler2, tb3_id, 11, 1, 1);
    CMD_SERIAL.println("    ok");
  }
}

void testMotor(uint8_t id)
{
  uint32_t pre_time;
  uint8_t  toggle = 0;

  if (id == 1)
  {
    CMD_SERIAL.printf("Test Motor Left...");
  }
  else
  {
    CMD_SERIAL.printf("Test Motor Right...");
  }
  // We run at 1000000
  portHandler->setBaudRate(1000000);

  uint16_t model_number;
  int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    CMD_SERIAL.printf(" found type: %d\n", model_number);
    write(portHandler, packetHandler2, id, 64, 1, 1);

    toggle = 0;
    pre_time = millis();
    write(portHandler, packetHandler2, id, 104, 4, 100);
    while (1)
    {
      if (CMD_SERIAL.available())
      {
        flushCmd();
        break;
      }

      if (millis() - pre_time > 1000)
      {
        pre_time = millis();

        toggle ^= 1;

        if (toggle)
        {
          write(portHandler, packetHandler2, id, 104, 4, 0);
        }
        else
        {
          write(portHandler, packetHandler2, id, 104, 4, 100);
        }
      }
    }
    write(portHandler, packetHandler2, id, 104, 4, 0);
  }
  else
  {
    CMD_SERIAL.printf("    dxl motor ID:%d not found\n", id);
  }
}

void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
    CMD_SERIAL.println("Fail to write!");
  }
}

dxl_ret_t read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;
  dxl_ret_t ret;

  int8_t  value8    = 0;
  int16_t value16   = 0;
  int32_t value32   = 0;


  if (length == 1)
  {
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));

    if (length == 1)
    {
      ret.u32Data = value8;
    }
    else if (length == 2)
    {
      ret.u32Data = value16;
    }
    else if (length == 4)
    {
      ret.u32Data = value32;
    }
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_error));
    CMD_SERIAL.println("Fail to read! ");
  }

  return ret;
}
