/* Dynamixel Reg Action 
 
 This example shows how to use reg/action instruction using new packet methods
 Open serial monitor and type 'a' on keyboard
 you can check moving dynamixel, type 'a' again -> moves its origin postion 0
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        X      X
 OpenCM9.04     O      O      O        X      X
  **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup(){
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
}
char temp;// need to receive from USB(PC)
uint32 GoalPos = 0;

void loop(){
  /*initPacket method needs ID and instruction*/
  Dxl.initPacket(1,INST_REG_WRITE);
  /* From now, insert byte data to packet without any index or data length*/
  Dxl.pushByte(30);
  Dxl.pushByte(DXL_LOBYTE(GoalPos));
  Dxl.pushByte(DXL_HIBYTE(GoalPos));
  /* just transfer packet to dxl bus without any arguments*/
  Dxl.flushPacket();
  if(Dxl.getResult()==(1<<COMM_RXSUCCESS) )
    SerialUSB.println("Comm Success");

  delay(100);
  if(SerialUSB.available()){
    temp = SerialUSB.read();
    if(temp == 'a'){
      SerialUSB.println("Reg Action");//Now start move dynamixel
      Dxl.initPacket(BROADCAST_ID,INST_ACTION);
      /* Action instruction does not need any push bytes*/
      Dxl.flushPacket();
      if( Dxl.getResult()==(1<<COMM_RXSUCCESS) ){
        SerialUSB.println("Comm Success");
        if(GoalPos == 0)
          GoalPos = 1023;
        else
          GoalPos = 0;
      }      
    }
  }
}

