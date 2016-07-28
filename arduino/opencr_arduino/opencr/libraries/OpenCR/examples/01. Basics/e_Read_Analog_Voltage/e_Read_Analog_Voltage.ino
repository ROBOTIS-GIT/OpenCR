/* Read_Analog_Voltage

 Reads an analog input on pin 1, prints the result to the serial monitor.  
 voltage = Analog_data * (Max_voltage / Max_Analog_data);
 
                  Compatibility
 CM900                  O
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

volatile float voltage=0;

void setup(){
  pinMode(1, INPUT_ANALOG);
}

void loop(){
  int AdcData = analogRead(1);
  voltage = AdcData * (3.3 / 4095.0);
  SerialUSB.print("voltage = ");
  SerialUSB.print(voltage);
  SerialUSB.println(" [V]");
  delay(100);
}

