/* Dynamixel Baud Change Example
 
 This example shows how to set baud rate and ID using broadcast ID
 All dynamixel in bus set to as 1Mbps , ID =1
 If you want your all dxl to reset ID and baud rate concurrentely.
 utilize this example.
 
 ID=0xfe is broadcast ID, refer to ROBOTIS E-manual
 
 The sequence is described as the below
 1. init dxl bus as 57600bps
 2. All dxls are set to be ID = 1
 3. New baud rate is set to be 1 Mbps
 4. After above changement, it is successfull if dxl moves well
 
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
#define GOAL_POSITION 30
#define BAUD_RATE 4

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(1);//dynamixel bus as 57600bps
  
  //AX MX RX Series
  Dxl.writeByte(BROADCAST_ID, 3, ID_NUM);  //set Dynamixel ID 1
  Dxl.writeByte(BROADCAST_ID, 4, 1);  //Baud rate set to 1 Mbps
  
  //XL-320 
  //Dxl.writeByte(ID_NUM, BAUD_RATE, 3);
  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
}

void loop() {
  // Wait for 0.5 second (500 milliseconds)
  delay(500);              
  /* Turn dynamixel ID 1 to position 0 on changed baud rate*/
  Dxl.writeWord(ID_NUM, GOAL_POSITION, 0); 
   // Wait for 0.5 second 
  delay(500);             
  /* Turn dynamixel ID 1 to position 1023 on changed baud rate*/
  Dxl.writeWord(ID_NUM, GOAL_POSITION, 1023);
}
