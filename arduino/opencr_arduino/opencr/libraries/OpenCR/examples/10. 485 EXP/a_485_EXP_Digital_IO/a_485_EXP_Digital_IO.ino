/* 
  485 EXP Digital IO
 
 
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


Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);

  pinMode(RED_LED_485EXP, OUTPUT); 
  pinMode(GREEN_LED_485EXP, OUTPUT); 
  pinMode(BLUE_LED_485EXP, OUTPUT); 
  pinMode(BUTTON1_485EXP, INPUT); 
  pinMode(BUTTON2_485EXP, INPUT); 
  Dxl.ledOff(1);
  Dxl.ledOff(2);
}
void loop() {
  if(digitalRead(BUTTON2_485EXP) == 1){
    delay(100);
    if(digitalRead(BUTTON2_485EXP) == 1){
      digitalWrite(RED_LED_485EXP, 0);
      digitalWrite(GREEN_LED_485EXP, 0);
      digitalWrite(BLUE_LED_485EXP, 0);
    }
  }
  else if(digitalRead(BUTTON1_485EXP) == 1){
    delay(100);
    if(digitalRead(BUTTON1_485EXP) == 1){
      digitalWrite(RED_LED_485EXP, 0);
      digitalWrite(GREEN_LED_485EXP, 0);
      digitalWrite(BLUE_LED_485EXP, 0);
    } 
  }
  else{
      digitalWrite(RED_LED_485EXP, 1);
      digitalWrite(GREEN_LED_485EXP, 1);
      digitalWrite(BLUE_LED_485EXP, 1);
  }
  for(int i = 1;i<=10;i++){
    Dxl.ledOff(i);
  }
  delay(80);

  for(int i = 1;i<=10;i++){
    Dxl.ledOn(i, 1);
  }
  delay(80);
}









