#include <DynamixelSDK.h>
#include <stdarg.h>


// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0

// Default setting
#define DEVICENAME                      "/dev/OpenCR"       // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"


#define CMD_SERIAL    Serial                                // USB Serial
//#define CMD_SERIAL    Serial2                               // Bluetooth Serial





void setup()
{
  CMD_SERIAL.begin(57600);
  while(!CMD_SERIAL);
}

void loop()
{
  dxl_monitor_main();
}


static void _printf(char *fmt, ... )
{
  char buf[128]; // resulting string limited to 128 chars
  char buf_out[128];
  va_list args;
  int i;
  int i_out;


  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);

  i_out = 0;
  for( i=0; i<strlen(buf); i++ )
  {
    if( buf[i] == '\n' )
    {
      buf_out[i_out++] = '\r';
      buf_out[i_out++] = '\n';
    }
    else
    {
      buf_out[i_out++] = buf[i];
    }
  }
  buf_out[i_out] = 0;

  CMD_SERIAL.print(buf_out);
}

static void _fprintf(FILE * stream, char *fmt, ... )
{
  char buf[128]; // resulting string limited to 128 chars
  char buf_out[128];
  va_list args;
  int i;
  int i_out;

  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);

  i_out = 0;
  for( i=0; i<strlen(buf); i++ )
  {
    if( buf[i] == '\n' )
    {
      buf_out[i_out++] = '\r';
      buf_out[i_out++] = '\n';
    }
    else
    {
      buf_out[i_out++] = buf[i];
    }
  }
  buf_out[i_out] = 0;

  CMD_SERIAL.print(buf_out);
}

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

char * fgets ( char * str, int num, FILE * stream )
{
  char ch;
  int  index;

  index = 0;

  while(1)
  {
    if( CMD_SERIAL.available() > 0 )
    {
      ch = CMD_SERIAL.read();

      CMD_SERIAL.write(ch);

      if( index < num-1 && ch != 0x0D )
      {
        str[index++] = ch;
      }
      if( ch == 0x0D)
      {
        str[index] = 0;
        break;
      }
    }
  }

  return str;
}

void usage(char *progname)
{
  _printf("-----------------------------------------------------------------------\n");
  _printf("Usage: %s\n");
  _printf(" [-h | --help]........: display this help\n");
  _printf("[-d | --device]......: port to open\n", progname);
  _printf("-----------------------------------------------------------------------\n");
}

void help()
{
  _printf("\n");
  _printf("                    .----------------------------.\n");
  _printf("                    |  DXL Monitor Command List  |\n");
  _printf("                    '----------------------------'\n");
  _printf(" =========================== Common Commands ===========================\n");
  _printf(" \n");
  _printf(" help|h|?                    :Displays help information\n");
  _printf(" baud [BAUD_RATE]            :Changes baudrate to [BAUD_RATE] \n");
  _printf("                               ex) baud 2400 (2400 bps) \n");
  _printf("                               ex) baud 1000000 (1 Mbps)  \n");
  _printf(" exit                        :Exit this program\n");
  _printf(" scan                        :Outputs the current status of all Dynamixels\n");
  _printf(" ping [ID] [ID] ...          :Outputs the current status of [ID]s \n");
  _printf(" bp                          :Broadcast ping (Dynamixel Protocol 2.0 only)\n");
  _printf(" \n");
  _printf(" ==================== Commands for Dynamixel Protocol 1.0 ====================\n");
  _printf(" \n");
  _printf(" wrb1|w1 [ID] [ADDR] [VALUE] :Write byte [VALUE] to [ADDR] of [ID]\n");
  _printf(" wrw1 [ID] [ADDR] [VALUE]    :Write word [VALUE] to [ADDR] of [ID]\n");
  _printf(" rdb1 [ID] [ADDR]            :Read byte value from [ADDR] of [ID]\n");
  _printf(" rdw1 [ID] [ADDR]            :Read word value from [ADDR] of [ID]\n");
  _printf(" r1 [ID] [ADDR] [LENGTH]     :Dumps the control table of [ID]\n");
  _printf("                               ([LENGTH] bytes from [ADDR])\n");
  _printf(" reset1|rst1 [ID]            :Factory reset the Dynamixel of [ID]\n");
  _printf(" \n");
  _printf(" ==================== Commands for Dynamixel Protocol 2.0 ====================\n");
  _printf(" \n");
  _printf(" wrb2|w2 [ID] [ADDR] [VALUE] :Write byte [VALUE] to [ADDR] of [ID]\n");
  _printf(" wrw2 [ID] [ADDR] [VALUE]    :Write word [VALUE] to [ADDR] of [ID]\n");
  _printf(" wrd2 [ID] [ADDR] [VALUE]    :Write dword [VALUE] to [ADDR] of [ID]\n");
  _printf(" rdb2 [ID] [ADDR]            :Read byte value from [ADDR] of [ID]\n");
  _printf(" rdw2 [ID] [ADDR]            :Read word value from [ADDR] of [ID]\n");
  _printf(" rdd2 [ID] [ADDR]            :Read dword value from [ADDR] of [ID]\n");
  _printf(" r2 [ID] [ADDR] [LENGTH]     :Dumps the control table of [ID]\n");
  _printf("                               ([LENGTH] bytes from [ADDR])\n");
  _printf(" reboot2|rbt2 [ID]           :reboot the Dynamixel of [ID]\n");
  _printf(" reset2|rst2 [ID] [OPTION]   :Factory reset the Dynamixel of [ID]\n");
  _printf("                               OPTION: 255(All), 1(Except ID), 2(Except ID&Baud)\n");

  _printf("\n");
}

void scan(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler1, dynamixel::PacketHandler *packetHandler2)
{
  uint8_t dxl_error;
  uint16_t dxl_model_num;

  _fprintf(stderr, "\n");
  _fprintf(stderr, "Scan Dynamixel Using Protocol 1.0\n");
  for (int id = 1; id < 253; id++)
  {
    if (packetHandler1-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
    {
      _fprintf(stderr, "\n                                          ... SUCCESS \r");
      _fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
    }
    else
        _fprintf(stderr, ".");

    if (kbhit())
    {
        char c = getch();
        if (c == 0x1b)
        break;
    }
  }
  _fprintf(stderr, "\n\n");

  _fprintf(stderr, "Scan Dynamixel Using Protocol 2.0\n");
  for (int id = 1; id < 253; id++)
  {
    if (packetHandler2-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
    {
      _fprintf(stderr, "\n                                          ... SUCCESS \r");
      _fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
    }
    else
    {
      _fprintf(stderr, ".");
    }

    if (kbhit())
    {
      char c = getch();
      if (c == 0x1b) break;
    }
  }
  _fprintf(stderr, "\n\n");
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
    if (dxl_error != 0) packetHandler->printRxPacketError(dxl_error);
    _fprintf(stderr, "\n Success to write\n\n");
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    _fprintf(stderr, "\n Fail to write! \n\n");
  }
}

void read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;

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
    if (dxl_error != 0) packetHandler->printRxPacketError(dxl_error);

    if (length == 1)
    {
      _fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint8_t)value8, value8);
    }
    else if (length == 2)
    {
      _fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint16_t)value16, value16);
    }
    else if (length == 4)
    {
      _fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint32_t)value32, value32);
    }
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    _fprintf(stderr, "\n Fail to read! \n\n");
  }
}

void dump(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t len)
{
  uint8_t  dxl_error       = 0;
  int      dxl_comm_result = COMM_TX_FAIL;
  uint8_t *data            = (uint8_t*)calloc(len, sizeof(uint8_t));

  dxl_comm_result = packetHandler->readTxRx(portHandler, id, addr, len, data, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
      packetHandler->printRxPacketError(dxl_error);

    if (id != BROADCAST_ID)
    {
      _fprintf(stderr, "\n");
      for (int i = addr; i < addr+len; i++)
      _fprintf(stderr, "ADDR %.3d [0x%.4X] :     %.3d [0x%.2X] \n", i, i, data[i-addr], data[i-addr]);
      _fprintf(stderr, "\n");
    }
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    _fprintf(stderr, "\n Fail to read! \n\n");
  }

  free(data);
}

void dxl_monitor_main(void)
{
  // Initialize Packethandler1 instance
  dynamixel::PacketHandler *packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

  // Initialize Packethandler2 instance
  dynamixel::PacketHandler *packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  _fprintf(stderr, "\n***********************************************************************\n");
  _fprintf(stderr,   "*                            DXL Monitor                              *\n");
  _fprintf(stderr,   "***********************************************************************\n\n");

  char *dev_name = (char*)DEVICENAME;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(dev_name);


  // Open port
  if (portHandler->openPort())
  {
    _printf("Succeeded to open the port!\r\n");
    _printf(" - Device Name : %s\r\n", dev_name);
    _printf(" - Baudrate    : %d\r\n", portHandler->getBaudRate());
  }
  else
  {
    _printf("Failed to open the port! [%s]\n", dev_name);
    _printf("Press any key to terminate...\n");
    getch();
    while(1);
  }

  _printf("Be sure to enable Carriage return\n\n");

  char    input[128];
  char    cmd[80];
  char    param[20][30];
  int     num_param;
  char    *token;
  uint8_t dxl_error;


  while(1)
  {
    _printf("[CMD] ");
    fgets(input, sizeof(input), stdin);
    char *p;
    if ((p = strchr(input, '\n'))!= NULL) *p = '\0';

    if (strlen(input) == 0) continue;

    token = strtok(input, " ");

    if (token == 0) continue;

    strcpy(cmd, token);
    token = strtok(0, " ");
    num_param = 0;
    while(token != 0)
    {
      strcpy(param[num_param++], token);
      token = strtok(0, " ");
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
    {
      help();
    }
    else if (strcmp(cmd, "baud") == 0)
    {
      if (num_param == 1)
      {
        if (portHandler->setBaudRate(atoi(param[0])) == false)
          _fprintf(stderr, " Failed to change baudrate! \n");
        else
          _fprintf(stderr, " Success to change baudrate! [ BAUD RATE: %d ]\n", atoi(param[0]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
        continue;
      }
    }
    else if (strcmp(cmd, "exit") == 0)
    {
      portHandler->closePort();
      return;
    }
    else if (strcmp(cmd, "scan") == 0)
    {
      scan(portHandler, packetHandler1, packetHandler2);
    }
    else if (strcmp(cmd, "ping") == 0)
    {
      uint16_t dxl_model_num;

      if (num_param == 0)
      {
        _fprintf(stderr, " Invalid parameters! \n");
        continue;
      }

      _fprintf(stderr, "\n");
      _fprintf(stderr, "ping Using Protocol 1.0\n");
      for (int i = 0; i < num_param; i++)
      {
        if (packetHandler1->ping(portHandler, atoi(param[i]), &dxl_model_num, &dxl_error) == COMM_SUCCESS)
        {
          _fprintf(stderr, "\n                                          ... SUCCESS \r");
          _fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", atoi(param[i]), dxl_model_num);
        }
        else
        {
          _fprintf(stderr, "\n                                          ... FAIL \r");
          _fprintf(stderr, " [ID:%.3d] \n", atoi(param[i]));
        }
      }
      _fprintf(stderr, "\n");

      _fprintf(stderr, "\n");
      _fprintf(stderr, "ping Using Protocol 2.0\n");
      for (int i = 0; i < num_param; i++)
      {
        if (packetHandler2->ping(portHandler, atoi(param[i]), &dxl_model_num, &dxl_error) == COMM_SUCCESS)
        {
          _fprintf(stderr, "\n                                          ... SUCCESS \r");
          _fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", atoi(param[i]), dxl_model_num);
        }
        else
        {
          _fprintf(stderr, "\n                                          ... FAIL \r");
          _fprintf(stderr, " [ID:%.3d] \n", atoi(param[i]));
        }
      }
      _fprintf(stderr, "\n");
    }
    else if (strcmp(cmd, "bp") == 0)
    {
      if (num_param == 0)
      {
        std::vector<unsigned char> vec;

        int dxl_comm_result = packetHandler2->broadcastPing(portHandler, vec);
        if (dxl_comm_result != COMM_SUCCESS) packetHandler2->printTxRxResult(dxl_comm_result);

        for (unsigned int i = 0; i < vec.size(); i++)
        {
          _fprintf(stderr, "\n                                          ... SUCCESS \r");
          _fprintf(stderr, " [ID:%.3d] \n", vec.at(i));
        }
        _printf("\n");
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrb1") == 0 || strcmp(cmd, "w1") == 0)
    {
      if (num_param == 3)
      {
        write(portHandler, packetHandler1, atoi(param[0]), atoi(param[1]), 1, atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrb2") == 0 || strcmp(cmd, "w2") == 0)
    {
      if (num_param == 3)
      {
        write(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 1, atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrw1") == 0)
    {
      if (num_param == 3)
      {
        write(portHandler, packetHandler1, atoi(param[0]), atoi(param[1]), 2, atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrw2") == 0)
    {
      if (num_param == 3)
      {
        write(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 2, atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "wrd2") == 0)
    {
      if (num_param == 3)
      {
        write(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 4, atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdb1") == 0)
    {
      if (num_param == 2)
      {
        read(portHandler, packetHandler1, atoi(param[0]), atoi(param[1]), 1);
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdb2") == 0)
    {
      if (num_param == 2)
      {
        read(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 1);
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdw1") == 0)
    {
      if (num_param == 2)
      {
        read(portHandler, packetHandler1, atoi(param[0]), atoi(param[1]), 2);
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdw2") == 0)
    {
      if (num_param == 2)
      {
        read(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 2);
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "rdd2") == 0)
    {
      if (num_param == 2)
      {
        read(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), 4);
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "r1") == 0)
    {
      if (num_param == 3)
      {
        dump(portHandler, packetHandler1, atoi(param[0]), atoi(param[1]), atoi(param[2]));
      }
      else{
        _fprintf(stderr, " Invalid parameters! \n");}
    }
    else if (strcmp(cmd, "r2") == 0)
    {
      if (num_param == 3)
      {
        dump(portHandler, packetHandler2, atoi(param[0]), atoi(param[1]), atoi(param[2]));
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reboot2") == 0 || strcmp(cmd, "rbt2") == 0)
    {
      if (num_param == 1)
      {
        int dxl_comm_result = packetHandler2->reboot(portHandler, atoi(param[0]), &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS)
        {
          if (dxl_error != 0) packetHandler2->printRxPacketError(dxl_error);
          _fprintf(stderr, "\n Success to reboot! \n\n");
        }
        else
        {
          packetHandler2->printTxRxResult(dxl_comm_result);
          _fprintf(stderr, "\n Fail to reboot! \n\n");
        }
      }
      else
      {
          _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reset1") == 0 || strcmp(cmd, "rst1") == 0)
    {
      if (num_param == 1)
      {
        int dxl_comm_result = packetHandler1->factoryReset(portHandler, atoi(param[0]), 0x00, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS)
        {
          if (dxl_error != 0)
            packetHandler1->printRxPacketError(dxl_error);
          _fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          packetHandler1->printTxRxResult(dxl_comm_result);
          _fprintf(stderr, "\n Fail to reset! \n\n");
        }
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else if (strcmp(cmd, "reset2") == 0 || strcmp(cmd, "rst2") == 0)
    {
      if (num_param == 2)
      {
        int dxl_comm_result = packetHandler2->factoryReset(portHandler, atoi(param[0]), atoi(param[1]), &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS)
        {
          if (dxl_error != 0) packetHandler2->printRxPacketError(dxl_error);
          _fprintf(stderr, "\n Success to reset! \n\n");
        }
        else
        {
          packetHandler2->printTxRxResult(dxl_comm_result);
          _fprintf(stderr, "\n Fail to reset! \n\n");
        }
      }
      else
      {
        _fprintf(stderr, " Invalid parameters! \n");
      }
    }
    else
    {
      _printf(" Bad command! Please input 'help'.\n");
    }
  }
}
