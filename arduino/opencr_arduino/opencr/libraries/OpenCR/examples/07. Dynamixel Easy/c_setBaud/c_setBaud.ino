/* Dynamixel Baud Change Example
 
 This example shows how to set baud rate and ID using broadcast ID
 All dynamixel in bus set to as 1Mbps , ID =1
 If you want your all dxl to reset ID and baud rate concurrentely.
 utilize this example.
 
 ID=0xfe is broadcast ID, refer to ROBOTIS E-manual
 
 
 The sequence is described as the below
 1. init dxl bus as 57600bps
 2. All dxls are set to be ID = 1
 3. New baud rate is set to be 1 Mbps
 4. After above changement, it is successfull if dxl moves well
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel Compatibility
               AX    MX     RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM 1
Dynamixel Dxl(DXL_BUS_SERIAL1); //Dynamixel on Serial1(USART1)

void setup() {
  // Initialize the dynamixel bus:

  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(1);  //57600bps
  Dxl.setID(BROADCAST_ID, ID_NUM); //All Dynamixel ID set to ID 1
  Dxl.setBaud(BROADCAST_ID, 1);  //All Dynamixel Baud rate set to 1 Mbps
  // Re-initialize dynamixel bus as 1Mbps
  Dxl.begin(3);  // initialize again as changed baud rate 1Mbps  
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
}

void loop() {
  delay(500);              // Wait for 0.5 second (500 milliseconds)
  Dxl.goalPosition(ID_NUM, 1); //Turn dynamixel ID 1 to position 1
  delay(500);              // Wait for 0.5 second 
  Dxl.goalPosition(ID_NUM, 1023);//Turn dynamixel ID 1 to position 1023
}





