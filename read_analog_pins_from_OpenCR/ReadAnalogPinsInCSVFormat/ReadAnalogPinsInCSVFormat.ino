/*
  ReadAnalogPinsInCSVFormat

  Reads analog inputs on pins A0-A5, prints the results in CSV format.
  This script is especially useful when its output is piped to a file and plotted using plot_opencr_sensors.py.
  Real-time graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).

  This script was adapted from the Arduino IDE's AnalogReadSerial example.
*/


const int numPins = 6;  // A0-A5 = 6 pins

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);  // OpenCR  // Serial.begin(9600);  // Arduino

  // // Set all pins to pulldown
  // for (int pin = 0; pin < numPins; pin++) {
  //   pinMode(A0 + pin, INPUT);
  // }
  
  // Print header labels for Serial Plotter (optional, helps with identification)
  // This will be the header row for your CSV
  // Serial.println("A3,A4,A5");  // 
  Serial.println("A0,A1,A2,A3,A4,A5");

  analogReadResolution(12);
}

// the loop routine runs over and over again forever:
void loop() {
  // read all analog pins A0-A5 and print comma-separated values
  for (int pin = 0; pin < numPins; pin++) {
    // retrieve measurement in bits
    int sensorValue = analogRead(A0 + pin);

    // // convert measurement to V
    float voltage = sensorValue / 4095.0 * 3.3;
    // // print the V
    Serial.print(voltage);

    // print just the sensor value
    // Serial.print(sensorValue);
    
    // add comma between values (except for the last one)
    if (pin < numPins - 1) {
      Serial.print(",");
    }
  }
  
  Serial.println(); // newline to complete the line
  delay(100); // small delay for stability (Serial Plotter can handle faster updates)
}
