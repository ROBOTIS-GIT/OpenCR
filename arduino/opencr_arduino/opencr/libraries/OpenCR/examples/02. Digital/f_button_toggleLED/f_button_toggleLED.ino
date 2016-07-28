/*Button Toggle LED
 
 Toggles the built-in LED using built-in button on OpenCM9.04
 repeatedly.
 
                    Compatibility
 CM900                  X
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup() {
  // Set up the built-in LED pin as an output:
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
}

void loop() {
  if(digitalRead(BOARD_BUTTON_PIN))
    toggleLED();

}

