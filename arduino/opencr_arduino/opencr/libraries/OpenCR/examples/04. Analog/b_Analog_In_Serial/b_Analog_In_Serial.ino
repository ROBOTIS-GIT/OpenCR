/* Analog input_Serial

  Reads an analog input pin, prints the results to the serial monitor.

  The circuit:
   Potentiometer connected to analog pin 1.
   Center pin of the potentiometer goes to the analog pin.
   Side pins of the potentiometer go to +3.3V (VCC) and ground

                   Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
*/

// Analog input pin.  You may need to change this number if your board
const int analogInputPin = 1;

void setup() {
  // Declare analogInputPin as INPUT_ANALOG:
  pinMode(analogInputPin, INPUT_ANALOG);
}

void loop() {
  // Read the analog input into a variable:
  int analogValue = analogRead(analogInputPin);

  // print the result:
  SerialUSB.println(analogValue);
  //need some delay because coming out too fast from USB COM port
  delay(100);
}
