/*Blink2(LED)
 
 Turns on the built-in(Status) LED on for 0.1 second, then off for 0.1 second,
 repeatedly using toggleLED() which is function only for built-in LED.
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O

 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup() {
  // Set up the built-in LED pin as an output:
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
  toggleLED();  //toggle digital value based on current value.
  delay(100);   //Wait for 0.1 second
}

