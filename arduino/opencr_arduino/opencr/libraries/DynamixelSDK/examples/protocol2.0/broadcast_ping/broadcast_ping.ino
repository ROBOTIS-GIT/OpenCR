#include <DynamixelSDK.h>


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

        
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

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number
  
  std::vector<uint8_t> vec;                       // Dynamixel data storages
  
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

  // Try to broadcast ping the Dynamixel
  dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

  Serial.print("Detected Dynamixel : \n");
  for (int i = 0; i < (int)vec.size(); i++)
  {
    Serial.print("ID : ");
    Serial.println(vec.at(i));
  }


  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}
