/* Dynamixel SyncWrite2

  Dynamixel SyncWrite exmaple using new packet methods
  This example shows same movement as previous syncwrite
  but, it made by new packet method, initPacket(), pushByte(), flushPacket()
  It does not need any complex length fomula and parameter index.
  After initPacket(), just push bytes you want to send to DXL bus and flushPacket().
  
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
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

#define NUM_ACTUATOR        5 // Number of actuator
#define CONTROL_PERIOD      (1000) // msec (Large value is more slow)
#define MAX_POSITION        1023
#define GOAL_SPEED          32
#define GOAL_POSITION       30

word  AmpPos = 512;
word  wPresentPos;
word  GoalPos = 0;
byte  id[NUM_ACTUATOR];
byte  i;

void setup() {
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //Insert dynamixel ID number to array id[]
  for(i=0; i<NUM_ACTUATOR; i++ ){
    id[i] = i+1;
  }
  //Set all dynamixels as same condition.
  Dxl.writeWord( BROADCAST_ID, GOAL_SPEED, 0 );
  Dxl.writeWord( BROADCAST_ID, GOAL_POSITION, AmpPos );
}

void loop() {
  /*initPacket method needs ID and instruction*/
  Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
  /* From now, insert byte data to packet without any index or data length*/
  Dxl.pushByte(GOAL_POSITION);
  Dxl.pushByte(2);
  //push individual data length per 1 dynamixel, goal position needs 2 bytes(1word)
 
  for( i=0; i<NUM_ACTUATOR; i++ ){
    Dxl.pushByte(id[i]);
    Dxl.pushByte(DXL_LOBYTE(GoalPos));
    Dxl.pushByte(DXL_HIBYTE(GoalPos));
    
    SerialUSB.println(GoalPos);
  }
  /* just transfer packet to dxl bus without any arguments*/
  Dxl.flushPacket();
  
  if(!Dxl.getResult()){
    SerialUSB.println("Comm Fail");
  }
  GoalPos += 100;
  
  if( GoalPos > MAX_POSITION )
    GoalPos -= MAX_POSITION;
  delay(CONTROL_PERIOD);

}
