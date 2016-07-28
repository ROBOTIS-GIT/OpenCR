/* AnalogReadSerial

 Reads an analog input on pin 0, prints the result to the serial monitor.
 This example code is in the public domain.
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O
 
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */



void setup(){
  pinMode(0, INPUT_ANALOG); // set pin 0 as analog input
}

void loop(){
  int AdcData = analogRead(0);
  SerialUSB.print("AdcData = ");
  SerialUSB.println(AdcData);
  delay(100);
}


