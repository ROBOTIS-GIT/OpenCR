/* LED_Fading
 
 This example shows how to fade an LED using the analogWrite() function.
 It operates only on OpenCM board, not CM-900(ES) because built-in led in CM900
 is not pwm-enabled pin, so you need some external circuit using other
 pwm-enabled pin.
 
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup() {  
  pinMode(BOARD_LED_PIN, PWM);  // setup the pin as PWM
}

void loop()  {
  // Fade in from min to max in increments of 1280 points:
  for (int fadeValue = 0; fadeValue <= 65535; fadeValue += 1280) {
    // Sets the value (range from 0 to 65535):
    analogWrite(BOARD_LED_PIN, fadeValue);
    // Wait for 30 milliseconds to see the dimming effect:
    delay(30);
  }

  // Fade out from max to min in increments of 1280 points:
  for (int fadeValue = 65535 ; fadeValue >= 0; fadeValue -= 1280) {
    // Sets the value (range from 0 to 1280):
    analogWrite(BOARD_LED_PIN, fadeValue);
    // Wait for 30 milliseconds to see the dimming effect:
    delay(30);
  }
}

