/* OLLO Gyro(Sensor) XY Read 

 connect Gyro Sensor Module(GS-12) to port 1 and 2.
 
 You can buy Gyro Sensor DYNAMIXEL in ROBOTIS-SHOP
 http://www.robotis-shop-en.com/shop/step1.php?number=833&b_code=B20070914051413&c_code=C20100528062452
 You can also find all information 
 http://support.robotis.com/
 
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#include <OLLO.h>
OLLO myOLLO;

void setup(){
  myOLLO.begin(1);//Gyro X Axis must be connected to port 1.
  myOLLO.begin(2);//Gyro Y Axis must be connected to port 2.
}
void loop(){
  SerialUSB.print("X-Axis ADC = ");
  SerialUSB.print(myOLLO.read(1)); //read X-Axis ADC value from OLLO port 1
  SerialUSB.print("  Y-Axis ADC = ");
  SerialUSB.println(myOLLO.read(2)); //read X-Axis ADC value from OLLO port 2
  delay(60);
}


