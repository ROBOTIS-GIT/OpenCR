/* Dynamixel Pro Basic Example
 
 Write 2 goal positions to ID 1 dynamixel pro
 turn left and right repeatly.
 Dynamixel pro use DXL protocol 2.0
 You can also find all information about DYNAMIXEL PRO and protocol 2.0
 http://support.robotis.com/
 
               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel check  the movement
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        X      O
 OpenCM9.04     X      X      X        X      O
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be 
 OpenCM 485EXP board ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM 1
#define LED_GREEN 564
#define GOAL_POSITION 596
#define TORQUE_ENABLE 562

Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //Toque on to move dynamixel pro
  Dxl.writeByte(ID_NUM,TORQUE_ENABLE,1);
}

void loop() {

  //Turn on green LED in DXL PRO
  Dxl.writeByte(ID_NUM,LED_GREEN,255);
  //Move to goal position 151875 refer to position limit
  Dxl.writeDword(ID_NUM,GOAL_POSITION,151875);
  delay(3000);
  //Turn off green LED in DXL PRO
  Dxl.writeByte(ID_NUM,LED_GREEN,0);
  //Move to goal position -151875 refer to position limit
  Dxl.writeDword(ID_NUM,GOAL_POSITION,-151875);

  delay(3000);
  //Read DXL internal temperature
  SerialUSB.print(" DXL PRO Temperature = ");
  SerialUSB.println(Dxl.readByte(ID_NUM,625));
}



