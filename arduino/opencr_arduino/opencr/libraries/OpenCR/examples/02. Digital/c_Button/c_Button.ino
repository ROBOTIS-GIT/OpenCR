/* Button
 
 OpenCM9.04 has built-in button which is called as user button
 User button is defined previously as BOARD_BUTTON PIN, so just call it.
 This example shows how to use user button and status LED(built-in LED)
 When button is pressed, status LED turn on, when released LED turn off
 
                  Compatibility
 CM900                  X
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup(){
  /*  
   BOARD_BUTTON_PIN is needed to pull-down circuit 
   for operationg as digital switch fully.
   */
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop(){
  // read the state of the pushbutton value:
  int buttonState = digitalRead(BOARD_BUTTON_PIN);

  if(buttonState==HIGH){ //if button is pushed, means 3.3V(HIGH) is connected to BOARD_BUTTON_PIN
    digitalWrite(BOARD_LED_PIN, LOW);
  }
  if(buttonState==LOW){// if button is released, means GND(LOW) is connected to BOARD_BUTTON_PIN
    digitalWrite(BOARD_LED_PIN, HIGH);
  }

}






