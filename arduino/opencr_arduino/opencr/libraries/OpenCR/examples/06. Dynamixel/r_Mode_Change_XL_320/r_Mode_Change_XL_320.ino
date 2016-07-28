/* Dynamixel Mode Change(XL_320)
 
                Compatibility
 CM900                  X
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        O      X
 OpenCM9.04     X      X      X        O      X
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

#define ID_NUM 1

int count = 1;
int flag = 0;
int mode_flag = 0;

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);

  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);
  attachInterrupt(BOARD_BUTTON_PIN,changeMode, RISING);
  myOLLO.begin(1, IR_SENSOR);

   wheel_change();

}

void changeMode(void){
  flag = 0;
  count = 1;                                                                                                                                                                                              
  if(mode_flag == 0) 
    mode_flag = 1;
  
  else if(mode_flag == 1)   
    mode_flag = 0;
  
  delay(50);
}

void loop() {
  if(mode_flag == 0){
    digitalWrite(BOARD_LED_PIN, 0);
    wheel_change();
    wheel_mode(); 
  }
  else if(mode_flag == 1){
    digitalWrite(BOARD_LED_PIN, 1 );
    position_change();
    position_mode();
  }
}

void wheel_change(){
  Dxl.writeByte(ID_NUM, 24, 0);
  Dxl.writeByte(ID_NUM, 11, 1);
  Dxl.writeByte(ID_NUM, 24, 1);
}

void position_change(){
  Dxl.writeByte(ID_NUM, 24, 0);
  Dxl.writeByte(ID_NUM, 11, 2);
  Dxl.writeByte(ID_NUM, 24, 1);
}

void wheel_mode(){
  Dxl.writeByte(ID_NUM, 25, count);
  Dxl.writeWord(ID_NUM, 32, 146*count);
  delay(1200);

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
  if(myOLLO.read(1, IR_SENSOR) >= 300){
          Dxl.writeWord(1, 32, 1023);
    if(flag == 0) flag = 1;
    else if(flag == 1) flag = 0;

    if(flag == 0)
      Dxl.writeWord(ID_NUM, 30, 1);

    else if(flag == 1)
      Dxl.writeWord(ID_NUM, 30, 1023);

    delay(1000);
    Dxl.writeByte(ID_NUM, 25, count);
    count++;
    if(count == 8) count = 1;
  }


}






