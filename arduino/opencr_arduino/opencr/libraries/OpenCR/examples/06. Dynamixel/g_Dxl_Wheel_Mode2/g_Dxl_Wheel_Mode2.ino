/* Dynamixel Wheel Mode 2 Example
 
 This example shows how to implement RC Car using two dynamixels
 The two dynamixels are set to as wheel mode by the way of changing
 CCW_Angle_Limit to 0
 If you want to use a dynamixel as joint mode, set CCW_Angle_Limit 
 to 0x3FF(1023) in AX-12A case
 
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
#define ID_Left_Wheel 1
#define ID_Right_Wheel 2

/* Control table defines */
#define CCW_Angle_Limit 8 //to change control mode
#define Goal_Postion 30
#define Moving_Speed 32

#define CONTROL_MODE 11

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  //AX MX RX Series  
  Dxl.writeByte(ID_Left_Wheel, CCW_Angle_Limit, 0);  //Dxl.jointMode(ID) can be used
  Dxl.writeByte(ID_Right_Wheel, CCW_Angle_Limit, 0);  
  
  //XL-320
  //Dxl.writeByte(ID_Left_Wheel, CONTROL_MODE, 1);  //Dxl.jointMode(ID) can be used
  //Dxl.writeByte(ID_Right_Wheel, CONTROL_MODE, 1);
}

void loop() {
  Dxl.writeWord(ID_Left_Wheel, Moving_Speed, 0); // stop at first
  Dxl.writeWord(ID_Right_Wheel, Moving_Speed, 0); 
  delay(1000);              // Wait for 1 sec

  Dxl.writeWord(ID_Left_Wheel, Moving_Speed, 300);// go ahead with velocity 300
  Dxl.writeWord(ID_Right_Wheel, Moving_Speed, 0x3FF | 300);
  delay(2000);              // Wait for 2 sec

  Dxl.writeWord(ID_Left_Wheel, Moving_Speed,  600); // speed up 
  Dxl.writeWord(ID_Right_Wheel, Moving_Speed, 0x3FF | 600);
  delay(2000);              // Wait for 2 sec
}


