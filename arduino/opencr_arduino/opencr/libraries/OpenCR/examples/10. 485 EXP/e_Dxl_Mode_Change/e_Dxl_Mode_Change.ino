/* Dynamixel Mode Change
 
                Compatibility
 CM900                  X
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
 #include <OLLO.h>
 OLLO myOLLO;
  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

/* 485 EXP switch, LED definces for 485 EXP device */
#define RED_LED_485EXP     18
#define GREEN_LED_485EXP   19
#define BLUE_LED_485EXP    20
#define BUTTON1_485EXP 16
#define BUTTON2_485EXP 17

/* Dynamixel ID defines */
#define ID_NUM 1


Dynamixel Dxl(DXL_BUS_SERIAL3);

int count = 1;
int flag = 0;
int mode_flag = 0;

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  pinMode(BUTTON1_485EXP, INPUT_PULLDOWN);
  pinMode(BUTTON2_485EXP, INPUT_PULLDOWN);
  pinMode(RED_LED_485EXP, OUTPUT); 
  pinMode(GREEN_LED_485EXP, OUTPUT); 
  pinMode(BLUE_LED_485EXP, OUTPUT); 

  attachInterrupt(BUTTON1_485EXP,wheel_mode_change, RISING);
  attachInterrupt(BUTTON2_485EXP,position_mode_change, RISING);
  myOLLO.begin(1, IR_SENSOR);
  
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.wheelMode(ID_NUM);
}

void wheel_mode_change(void){
  digitalWrite(BOARD_LED_PIN, 0);
  count = 1;            
  mode_flag = 0;
  Dxl.wheelMode(ID_NUM);
}

void position_mode_change(void){
  digitalWrite(BOARD_LED_PIN, 1);
  count = 1;            
  mode_flag = 1;
  Dxl.jointMode(ID_NUM);
} 

void loop() {
  if(mode_flag == 0){
    digitalWrite(BOARD_LED_PIN, 0);
    wheel_mode(); 
  }
  else if(mode_flag == 1){
    digitalWrite(BOARD_LED_PIN, 1 );
    position_mode();
  }
}




void wheel_mode(){
  Dxl.goalSpeed(ID_NUM, 146*count);
  delay(1500);

  if(count == 1)
    LED(0, 1, 1);
  if(count == 4)
    LED(0, 0, 1);
  if(count == 6)
    LED(0, 0, 0); 

  if(flag == 0){
    count++;
    if(count >= 7)
      flag = 1;
  }
  else if(flag == 1){
    count--;
    if(count <= 1)
      flag = 0; 
  }
}

void position_mode(){
  if(myOLLO.read(1, IR_SENSOR) < 50)
    Dxl.goalSpeed(ID_NUM, 100);
  else
    Dxl.goalSpeed(ID_NUM,(myOLLO.read(1, IR_SENSOR))*2);


  if(flag == 0) flag = 1;
  else if(flag == 1) flag = 0;

  if(flag == 0){
    LED(0, 0, 0);
    Dxl.goalPosition(ID_NUM,1);
  }
  else if(flag == 1){               
    LED(1, 1, 1);    
    Dxl.goalPosition(ID_NUM,1023);
  }
  delay(1500);

  count++;


  if(count == 8) count = 1;
}

void LED(int num, int nnum1, int nnum2){
  if(num == 1)
    digitalWrite(BLUE_LED_485EXP, 1);
  else
    digitalWrite(BLUE_LED_485EXP, 0);

  if(nnum1 == 1)
    digitalWrite(GREEN_LED_485EXP, 1);
  else
    digitalWrite(GREEN_LED_485EXP, 0);

  if(nnum2 == 1)
    digitalWrite(RED_LED_485EXP, 1);
  else
    digitalWrite(RED_LED_485EXP, 0);

  delay(10);
}









