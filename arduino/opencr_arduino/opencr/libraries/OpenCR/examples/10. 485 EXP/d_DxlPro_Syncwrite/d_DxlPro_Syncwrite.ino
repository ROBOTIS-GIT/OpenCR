/* Dynamixel Syncwrite Example
 
 Write goal position with velocity to ID 1 dynamixel pro.
 Dynamixel pro use DXL protocol 2.0. 
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
#define GOAL_POSITION 596
#define TORQUE_ENABLE 562

Dynamixel Dxl(DXL_BUS_SERIAL3);

int itemp1[3];
int itemp2[3];
void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.writeByte(ID_NUM,TORQUE_ENABLE,1);


}
void loop(){

  itemp1[0] = 1;
  itemp1[1] = 151875; //goal posiotion 1
  itemp1[2] = 10000;  //velocity 1

  itemp2[0] = 1;
  itemp2[1] = -151875; //goal position 2
  itemp2[2] = 5000;  //velocity 2
  Dxl.syncWrite(GOAL_POSITION,2,itemp1,3);
  delay(3000);
  Dxl.syncWrite(GOAL_POSITION,2,itemp2,3);
  delay(3000);
}


