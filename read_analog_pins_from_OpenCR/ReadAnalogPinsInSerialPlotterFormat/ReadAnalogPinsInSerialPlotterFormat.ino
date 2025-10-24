/*
  ReadAnalogPinsInSerialPlotterFormat

  Reads analog inputs on pins A0-A5, prints the results in the Arduino IDE's Serial Plotter format.
  Real-time graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).

  This script was adapted from the Arduino IDE's AnalogReadSerial example.
*/


const int numPins = 6;  // A0-A5 = 6 pins

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Print header labels for Serial Plotter (optional, helps with identification)
  Serial.println("A0,A1,A2,A3,A4,A5");
}

// the loop routine runs over and over again forever:
void loop() {
  // read all analog pins A0-A4 and print comma-separated values
  for (int pin = 0; pin < numPins; pin++) {
    // retrieve measurement
    int sensorValue = analogRead(A0 + pin);

    // print labeled value to serial console
    Serial.print("A");
    Serial.print(pin);
    Serial.print(":");
    Serial.print(sensorValue);
    
    // add comma between values (except for the last one)
    if (pin < numPins - 1) {
      Serial.print(",");
    }
  }
  
  Serial.println(); // newline to complete the line
  delay(10); // small delay for stability (Serial Plotter can handle faster updates)
}
