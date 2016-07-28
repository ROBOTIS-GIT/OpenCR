/* Dynamixel goalPosition
 
 Turns left the dynamixel , then turn right for one second,
 repeatedly.
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                   Dynamixel Compatibility
                AX    MX      RX    XL-320    Pro
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

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);  
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
}

void loop() {
  Dxl.goalPosition(ID_NUM, 1); //ID 1 dynamixel moves to position 1
  delay(2000);
  Dxl.goalPosition(ID_NUM, 1023);//ID 1 dynamixel moves to position 1023
  delay(2000);
}



