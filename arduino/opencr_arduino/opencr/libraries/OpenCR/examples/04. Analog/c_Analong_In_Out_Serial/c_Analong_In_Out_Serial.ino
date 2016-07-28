/* Analong_In_Out_Serial
 
 Demonstrates analog input by reading an analog sensor on analog pin
 0 and turning on and off the status LED(Light Emitting Diode).
 The amount of time the LED will be on and off depends on the
 value obtained by analogRead().
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

int sensorPin = 0;   // Select the input pin for the potentiometer
int sensorValue = 0; // Variable to store the value coming from the sensor

void setup() {
  // Declare the sensorPin as INPUT_ANALOG:
  pinMode(sensorPin, INPUT_ANALOG);
  // Declare the LED's pin as an OUTPUT.  (BOARD_LED_PIN is a built-in
  // constant which is the pin number of the built-in LED.  On the
  // Maple, it is 13.)
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
  // Read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  // Turn the LED pin on:
  digitalWrite(BOARD_LED_PIN, HIGH);
  // Stop the program for <sensorValue> milliseconds:
  delay(sensorValue);
  // Turn the LED pin off:
  digitalWrite(BOARD_LED_PIN, LOW);
  // Stop the program for for <sensorValue> milliseconds:
  delay(sensorValue);
}

