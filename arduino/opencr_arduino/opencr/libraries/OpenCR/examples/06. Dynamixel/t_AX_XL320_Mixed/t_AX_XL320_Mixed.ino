/* Dynamixel AX XL320 Mixed
 
 In this example, you can see how to use dxl protocol 1.0 and 2.0 at same time.
 New setPacketType() method makes this possible.
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
                   Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      X      X        O      X
 OpenCM9.04     O      X      X        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM 1
#define XL_320_ID_NUM 2

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  Dxl.setPacketType(DXL_PACKET_TYPE1);
  Dxl.jointMode(1);
  Dxl.setPacketType(DXL_PACKET_TYPE2);
  Dxl.jointMode(2);
}

void loop() {
  // put your main code here, to run repeatedly: 
  Dxl.setPacketType(DXL_PACKET_TYPE1);
  Dxl.goalPosition(ID_NUM, 1);
  delay(1000);
  Dxl.goalPosition(ID_NUM, 1023);
  delay(1000);
  
    Dxl.setPacketType(DXL_PACKET_TYPE2);
  Dxl.goalPosition(XL_320_ID_NUM, 1);
  delay(500);
  Dxl.goalPosition(XL_320_ID_NUM, 1023);
  delay(500);
}



