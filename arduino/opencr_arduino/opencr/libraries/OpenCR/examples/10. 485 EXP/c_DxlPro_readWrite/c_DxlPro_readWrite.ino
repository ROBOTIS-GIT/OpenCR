/* Dynamixel read/write Example

  Reads an dynaixel current movement, and set goal position
  turn left and right repeatly.
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
#define TORQUE_ENABLE 562
#define GOAL_POSITION 596
#define MOVING 610

Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //Toque is enable to move dynamixel pro
  Dxl.writeByte(ID_NUM,TORQUE_ENABLE,1);
}
byte isMoving = 0;
int goalPosition = 0;
void loop() {

  //Check if ID 1 dynamixel is still moving, address 610
  //Please refer ROBOTIS support page 
  //Check if the last communication is successful
  if(!Dxl.readByte(ID_NUM,MOVING)){
    
 //Send instruction packet to move for goalPosition( control address is 596 )
    Dxl.writeDword(ID_NUM,GOAL_POSITION, goalPosition );
     //toggle the position
    if(goalPosition == 151875)
      goalPosition = -151875;
    else
      goalPosition = 151875;
    
  }  
  delay(500);
}


