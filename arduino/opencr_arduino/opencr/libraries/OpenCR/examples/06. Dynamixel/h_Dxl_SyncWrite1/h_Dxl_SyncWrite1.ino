/*  Dynamixel SyncWrite
 
 This example shows same movement as previous syncwrite
 
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
/* Dynamixel ID defines */
#define ID_NUM_1  1
#define ID_NUM_2  2
#define ID_NUM_3  3

/* Control table defines */
#define P_GOAL_POSITION    30
#define P_GOAL_SPEED    32

/********* Sync write data  **************
 * ID1, DATA1, DATA2..., ID2, DATA1, DATA2,...
 ******************************************
 */
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

word SyncPage1[9]=
{ 
  ID_NUM_1,0,100,  // 3 Dynamixels are move to position 0
  ID_NUM_2,0,100,  // with velocity 100
  ID_NUM_3,0,100}; 
word SyncPage2[9]=
{ 
  ID_NUM_1,512,500, // 3 Dynamixels are move to position 512
  ID_NUM_2,512,500, // with velocity 500
  ID_NUM_3,512,500};

void setup(){
// Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //Set all dynamixels as same condition.
  Dxl.writeWord( BROADCAST_ID, P_GOAL_POSITION, 0 );
  Dxl.writeWord( BROADCAST_ID, P_GOAL_SPEED, 0 );
}

void loop(){
/*
 * byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
 */
  Dxl.syncWrite(30,2,SyncPage1,9);
  delay(1000);
  Dxl.syncWrite(30,2,SyncPage2,9);
  delay(1000);
}




