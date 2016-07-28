/* EXIT_Interrupt_LED
 
   Toggle Button must be conneted Pin 5 with pull-down 3.3V

   This example demonstrate how to use EXIT Interrupt with LED blinking.
   If pushed again, LED blinking

                  Compatibility
 CM900                  X
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
*/

#define LED_ON 0
#define LED_OFF 1

void setup(){
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  /*It can be choose as CHANGE, RISING or FALLING*/
  attachInterrupt(BOARD_BUTTON_PIN, changeDirection, RISING);
}

void changeDirection(void){
    digitalWrite(BOARD_LED_PIN, LED_ON);
    delay(100);
}

void loop(){
  digitalWrite(BOARD_LED_PIN, LED_OFF);
}
