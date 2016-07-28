/* IR Mouse
  
  Using 2 dynamixels and 2 IR sensors, you can make mouse robot to avoid obstacles.

                Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel Compatibility
               Ax    MX      Rx    XL-320    Pro
 CM900          O      X      X        X      X
 OpenCM9.04     O      X      X        X      X
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be OpenCM 485EXP board ****
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
*/
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define ID_NUM1 1
#define ID_NUM2 2
/* Control table defines */
#define GOAL_SPEED 32

#include <OLLO.h>
OLLO myOLLO;

Dynamixel Dxl(DXL_BUS_SERIAL1);

int defaultSpeed = 1000;
void setup(){
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  Dxl.wheelMode(ID_NUM1);//set id 1 as wheel mode
  Dxl.wheelMode(ID_NUM2);//set id 2 as wheel mode
  myOLLO.begin(1, IR_SENSOR);//IR Module must be connected at port 1.
  myOLLO.begin(4, IR_SENSOR);//IR Module must be connected at port 4.
}
int leftIr=0;
int rightIr = 0;
void loop(){
  /*SerialUSB.print("Left ADC = ");
  SerialUSB.print(myOLLO.read(1, IR_SENSOR)); //read ADC value from OLLO port 1
  SerialUSB.print("    Right ADC = ");
  SerialUSB.println(myOLLO.read(4, IR_SENSOR)); //read ADC value from OLLO port 1
 */
  leftIr = myOLLO.read(1, IR_SENSOR);
  rightIr = myOLLO.read(4, IR_SENSOR);
   if( leftIr > 100){//If an obstacle is detected in the left
    Dxl.writeWord(ID_NUM1,GOAL_SPEED, defaultSpeed );//change direction of ID 1 DXL
    delay(500);
  }
  else if( rightIr > 100){ //If an obstacle is detected in the right
    Dxl.writeWord(ID_NUM2,GOAL_SPEED, defaultSpeed | 0x400);//change direction of ID 2 DXL
    delay(500);
  }
  Dxl.writeWord(ID_NUM1,GOAL_SPEED, defaultSpeed | 0x400);
  Dxl.writeWord(ID_NUM2,GOAL_SPEED, defaultSpeed);
  delay(60);
}

