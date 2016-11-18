#include <DynamixelSDK.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_PRESENT_POSITION         4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


        
void setup() {
  // put your setup code here, to run once:
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
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[4];
  int32_t dxl1_present_position = 0, dxl2_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
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
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);

  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);

  while(1)
  {
    Serial.print("Press any key to continue! (or press q to quit!)\n");


    while(Serial.available()==0);

    int ch;

    ch = Serial.read();
    if( ch == 'q' )
      break;

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Syncread present position
      dxl_comm_result = groupSyncRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

      // Check if groupsyncread data of Dynamixel#1 is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Check if groupsyncread data of Dynamixel#2 is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Get Dynamixel#1 present position value
      dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Get Dynamixel#2 present position value
      dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      Serial.print("[ID:");      Serial.print(DXL1_ID); 
      Serial.print(" GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print(" PresPos:");  Serial.print(dxl1_present_position);
      Serial.print(" [ID:");      Serial.print(DXL2_ID); 
      Serial.print(" GoalPos:"); Serial.print(dxl_goal_position[index]);
      Serial.print(" PresPos:");  Serial.print(dxl2_present_position);
      Serial.println(" ");
//      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position);

    }while((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

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

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
