/* OLLO DMS Sensor Read 
 
 connect DMS Sensor Module(DMS-80) to port 1.
 
 You can buy DMS Sensor DYNAMIXEL in ROBOTIS-SHOP
 http://www.robotis-shop-en.com/shop/step1.php?number=834&b_code=B20070914051413&c_code=C20100528062452
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
  myOLLO.begin(1);//DMS Module must be connected at port 1.
}
void loop(){
  Serial.print("DMS Sensor ADC Value = ");
  Serial.println(myOLLO.read(1)); //read ADC value from OLLO port 1
  delay(60);
}



