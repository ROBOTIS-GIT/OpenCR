/* Dynamixel Wheel Mode Example
 
 This example shows how to use dynamixel as wheel mode
 All dynamixels are set as joint mode in factory,
 but if you want to make a wheel using dynamixel, 
 you have to change it to wheel mode by change CCW angle limit to 0
 
               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define ID_NUM 1
/* Control table defines */
#define GOAL_SPEED 32
#define CCW_Angle_Limit 8
#define CONTROL_MODE 11

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  
  //AX MX RX Series
  Dxl.writeWord(ID_NUM, CCW_Angle_Limit, 0); 
  //disable CCW Angle Limit(L) to use wheel mode
  
  //XL-320
  //Dxl.writeByte(ID_NUM, CONTROL_MODE, 1);
}

void loop() {
  //forward
  Dxl.writeWord(ID_NUM, GOAL_SPEED, 400); 
  delay(5000);
  //reverse
  Dxl.writeWord(ID_NUM, GOAL_SPEED, 400 | 0x400);
  delay(5000); 
  //stop
  Dxl.writeWord(ID_NUM, GOAL_SPEED, 0); 
  delay(2000);
}


