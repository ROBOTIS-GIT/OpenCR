/*EXIT_Interrupt_SerialUSB

  This example demonstrate how to use EXIT Interrupt with toggle LED blinking.
  EXIT_Interrupt! the results to the serial monitor.
  
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
*/

#define LED_ON 0
#define LED_OFF 1

void setup(){
  SerialUSB.begin();
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  /*It can be choose as CHANGE, RISING or FALLING*/
  attachInterrupt(BOARD_BUTTON_PIN,changeDirection, RISING);
}

void changeDirection(void){
  SerialUSB.print("EXIT_Interrupt!\r\n");
}

void loop(){
  toggleLED();
  delay(100);
}
