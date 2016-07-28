/* 
  Convert angle to goal position
 
  This example demonstrates how to convert angle degree to dxl goal position data
  In this way you are with reference to the mathematical formula may be applied to DXL.
                Compatibility
 CM900                  X
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        X      O
 OpenCM9.04     X      X      X        X      O
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
 
   /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485 EXP
 
#define ID_NUM1 1
#define ID_NUM2 2
#define ID_NUM3 3
#define TORQUE_ENABLE 562
#define GOAL_POSITION 596

 Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  Dxl.writeByte(BROADCAST_ID,TORQUE_ENABLE,1);
}

int goal  = 0;
int spd   = 0;
int spd_1 = 13;
int spd_2 = 27;

void loop() {
  
  goal = angle('a', 120);
  spd = speed('a', spd_2);
  Dxl.writeDword(ID_NUM1,600,spd);
  Dxl.writeDword(ID_NUM1,GOAL_POSITION,goal);
  goal = angle('b', 120);
  spd = speed('b', spd_2);
  Dxl.writeDword(ID_NUM2,600,spd);
  Dxl.writeDword(ID_NUM2,GOAL_POSITION,goal);
  goal = angle('c', 120);
  spd = speed('c', spd_2);
  Dxl.writeDword(ID_NUM3,600,spd);
  Dxl.writeDword(ID_NUM3,GOAL_POSITION,goal);
  delay(5000);   

  goal = angle('a', -120);
  spd = speed('a', spd_1);
  Dxl.writeDword(ID_NUM1,600,spd);
  Dxl.writeDword(ID_NUM1,GOAL_POSITION,goal);
  goal = angle('b', -120);
  spd = speed('b', spd_1);
  Dxl.writeDword(ID_NUM2,600,spd);
  Dxl.writeDword(ID_NUM2,GOAL_POSITION,goal);
  goal = angle('c', -120);
  spd = speed('c', spd_1);
  Dxl.writeDword(ID_NUM3,600,spd);
  Dxl.writeDword(ID_NUM3,GOAL_POSITION,goal);
  delay(5000);  
}


int angle(char model, int value_angle){
  int value = 0;
  if(model == 'a'){
    value = ((value_angle * 103860) / 180);
    return value;
  }
  else if(model == 'b'){
    value = ((value_angle * 125700) / 180);
    return value;
  }
  else if(model == 'c'){
    value = ((value_angle * 251000) / 180);
    return value;
  }
}

int speed(char model, int value_speed){
  int value = 0;
  if(model == 'a'){
    return (value_speed * 290);
  }
  else if(model == 'b'){
    return (value_speed * 250);
  }
  else if(model == 'c'){
    return (value_speed * 500);
  } 
}



