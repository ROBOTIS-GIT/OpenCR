/* Dynamixel ReadWrite
 
 Reads an dynaixel current position, and set goal position
 turn left and right repeatly.
 
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
#define ID_NUM  1
/* Control table defines */
#define GOAL_POSITION 30
#define MOVING 46
#define XL_MOVING 49

Dynamixel Dxl(DXL_BUS_SERIAL1);

byte isMoving = 0;
int goalPosition = 0;

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
}

void loop() {
  //Check if ID 1 dynamixel is still moving, 46 is moving control address
  //Please refer ROBOTIS support page
  
  //AX MX RX Series
  isMoving = Dxl.readByte(ID_NUM, MOVING);
  
  //XL320
  //isMoving = Dxl.readByte(ID_NUM, XL_MOVING);
  
  //Check if the last communication is successful
  if( isMoving == 0 ){ //if ID 1 dynamixel is stopped
  
    //Send instruction packet to move for goalPosition( control address is 30 )
    //Compatible with all dynamixel serise
    Dxl.writeWord(ID_NUM, GOAL_POSITION, goalPosition );
    //toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
    if(goalPosition == 1000)
      goalPosition = 0;
    else
      goalPosition = 1000;
  }
}

