/* Goal Acceleration
 
  In this example, you can see how to control goal acceleration in dxl pro.
  
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
 
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

/* 485 EXP switch, LED definces for 485 EXP device */
#define RED_485EXP     18
#define GREEN_485EXP   19
#define BLUE_485EXP    20
#define BUTTON1_485EXP 16
#define BUTTON2_485EXP 17

#define ID_NUM 1
#define TORQUE_ENABLE 562
#define GOAL_POSITION 596
#define BAUD_RATE 8
#define PRESENT_POSITION 611
Dynamixel Dxl(DXL_BUS_SERIAL3);

/* Minimum_Source*/
int count = 0;
int ccount = 0;
int stop_flag = 0;

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  pinMode(RED_485EXP, OUTPUT); 
  pinMode(GREEN_485EXP, OUTPUT); 
  pinMode(BLUE_485EXP, OUTPUT); 
  pinMode(BUTTON1_485EXP, INPUT); 
  pinMode(BUTTON2_485EXP, INPUT); 
  pinMode(BOARD_LED_PIN,OUTPUT);
  
  digitalWrite(RED_485EXP, 1);
  digitalWrite(GREEN_485EXP, 1);
  digitalWrite(BLUE_485EXP, 1);
  
  Dxl.writeByte(ID_NUM, BAUD_RATE, 1);
  Dxl.writeByte(ID_NUM, BAUD_RATE, 1);
  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps   
  Dxl.begin(1);
  
  //p gain setting
  Dxl.writeWord(ID_NUM, 594, 64);

  //Goal Position
  //Dxl.writeDword(ID_NUM, GOAL_POSITION, 0);

  //Goal Accelation
  Dxl.writeDword(ID_NUM, 606, 16);
  delay(1000);


  unsigned int a = 0;
  while(1){
    if((int32)Dxl.readDword(1, PRESENT_POSITION) > -10000 && (int32)Dxl.readDword(1, PRESENT_POSITION) < 10000)
     break;
  }
  toggleLED();
  
 Dxl.writeByte(ID_NUM, TORQUE_ENABLE, 1);
}

void loop() {
  // put your main code here, to run repeatedly: 
  stop();
  stop_flag = 0;
  if(digitalRead(BUTTON1_485EXP) == 1){
    delay(100);
    if(digitalRead(BUTTON1_485EXP) == 1){
      delay(1000);
      while(1){
        Dxl.writeDword(1, GOAL_POSITION, 62750);

        digitalWrite(GREEN_485EXP, 0);
        delay(100);
        while(abs((int32)Dxl.readDword(1, PRESENT_POSITION) - 62750) < 30 ){

          stop();
        }
        digitalWrite(GREEN_485EXP, 1);

        delayCount();
        if(stop_flag == 1)
          break;
        Dxl.writeDword(1, GOAL_POSITION, -62750);

        digitalWrite(GREEN_485EXP, 0);
        delay(100);
        while(abs((int32)Dxl.readDword(1, PRESENT_POSITION)- (-62750)) < 30 ){
          stop();
        }      
        digitalWrite(GREEN_485EXP, 1);
        delayCount();
        if(stop_flag == 1)
          break;
      }
    }
  }
}

void delayCount(){
  for(ccount = 0; ccount<30;ccount++){
    for(count = 0;count<65535;count++){
      stop();
    }
    stop(); 
  }
}

void stop(){
  if(digitalRead(BUTTON2_485EXP) == 1){
    delay(100);
    if(digitalRead(BUTTON2_485EXP) == 1){
      delay(2000);
      Dxl.writeDword(ID_NUM, GOAL_POSITION, 0);
      delay(1500);
      stop_flag = 1;
    }
  } 
}