/* EXIT_Interrupt_Dynamixel

 This example demonstrate how to use EXIT Interrupt with Dynamixel.
 If button is pushed, dynamixel connected moves to position 0. 
 If pushed again, it moves to position 500 repeatedly with LED blinking

 Toggle Button must be conneted Pin 5 with pull-down 3.3V
 
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               Ax     MX      Rx    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#define ID_Num 1

/* Address_Number_Define */
#define Goal_Postion 30
#define Moving_Speed 32

#define LED_ON 0
#define LED_OFF 1

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM 1

Dynamixel Dxl(DXL_BUS_SERIAL1);

volatile unsigned ExitFlag=1;

void setup(){

  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  /*It can be choose as CHANGE, RISING or FALLING*/
  attachInterrupt(BOARD_BUTTON_PIN,changeDirection, RISING);
  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  Dxl.jointMode(ID_NUM); //jointMode() to use position mode
}

void changeDirection(void){
  SerialUSB.print("Dxl_!\r\n");
  ExitFlag=1; 
}

void loop(){
  toggleLED();
  delay(100);

  if(ExitFlag==1)
  { 
    Dxl.writeWord(ID_Num, Goal_Postion, 0); //Turn dynamixel ID 1 to position 0
    delay(500);              // Wait for 1 second (1000 milliseconds)
    Dxl.writeWord(ID_Num, Goal_Postion, 300);//Turn dynamixel ID 1 to position 300
    delay(500);              // Wait for 1 second (1000 milliseconds)

    ExitFlag=0;
  }
}

