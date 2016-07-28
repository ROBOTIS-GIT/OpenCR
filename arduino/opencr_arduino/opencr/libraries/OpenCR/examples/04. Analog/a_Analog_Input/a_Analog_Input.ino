/* Analog input
 
 Reads an analog input pin, maps the result to a range from 0 to
 65535 and uses the result to set the pulse width modulation (PWM) of
 an output pin.  Also prints the results to the serial monitor.

 The circuit:
 * Potentiometer connected to analog pin 1.
 Center pin of the potentiometer goes to the analog pin.
 Side pins of the potentiometer go to +3.3V and ground.
 * LED connected from digital pin 14 to 3.3V( defined as BOARD_LED_PIN )
 If you have CM-900, you have to connect LED to any PWM enable pin.
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

const int analogInPin = 2; // Analog input pin that the potentiometer

// These variables will change:
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM

void setup() {
  // Configure the ADC pin
  pinMode(analogInPin, INPUT_ANALOG);
  // Configure LED pin
  pinMode(BOARD_LED_PIN, PWM);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 4095, 0, 65535);
  // change the analog out value:
  analogWrite(BOARD_LED_PIN, outputValue);

  // print the results to the serial monitor:
  SerialUSB.print("sensor = " );
  SerialUSB.print(sensorValue);
  SerialUSB.print("\t output = ");
  SerialUSB.println(outputValue);
  delay(100);
}


