/*Blink without delay
 
 Turns on and off the built-in light emitting diode (LED), without
 using the delay() function.  This means that other code can run at
 the same time without being interrupted by the LED code.
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O

 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
/*
millis()
 : Returns the number of milliseconds since the Arduino board began running the current program. 
 This number will overflow (go back to zero), after approximately 50 days.
 */

// Variables:
long previousMillis = 0; // will store the last time the LED was updated
int interval = 1000;  // interval at which to blink (in milliseconds)

void setup() {
  // Set up the built-in LED pin as output:
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
  // Check to see if it's time to blink the LED; that is, if the
  // difference between the current time and last time we blinked
  // the LED is bigger than the interval at which we want to blink

  if ((int)millis() - previousMillis > interval) {
    // Save the last time you blinked the LED
    previousMillis = millis();

    // If the LED is off, turn it on, and vice-versa:
    toggleLED();
  }
}

