/* OLLO TPS Sensor Read
 
 connect Temperature Sensor to OLLO ports 

                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#include <OLLO.h>
OLLO myOLLO;

#define YOUR_OLLO_PORT 3

void setup(){
  myOLLO.begin(YOUR_OLLO_PORT);// A Module needs to be connected to proper port (PORT 1 to 4)
}
void loop(){
  Serial.print("RAW Temperature = ");
  Serial.print(myOLLO.read(YOUR_OLLO_PORT)); //read ADC value from YOUR_OLLO-PORT
  Serial.print("\t Converted Temperature = ");
  Serial.println(myOLLO.read(YOUR_OLLO_PORT, TEMPERATURE_SENSOR)); //read ADC value from YOUR_OLLO-PORT
  delay(100);
}



