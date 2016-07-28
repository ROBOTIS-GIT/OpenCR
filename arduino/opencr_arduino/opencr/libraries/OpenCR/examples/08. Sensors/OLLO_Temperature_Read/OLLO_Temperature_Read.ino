/* OLLO Temperature Sensor Read 

 connect Temperature Sensor Module to port 1.
 
 You can buy Temperature Sensor DYNAMIXEL in ROBOTIS-SHOP
 http://www.robotis-shop-en.com/shop/step1.php?number=750&b_code=B20070914051413&c_code=C20100528062452
 //위 링크 수정
 
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
  myOLLO.begin(1, TEMPERATURE_SENSOR);//Temperature Module must be connected at port 1.
}
void loop(){
  SerialUSB.print("Temperature Sensor = ");
  SerialUSB.println(myOLLO.read(1, TEMPERATURE_SENSOR)); //read ADC value from OLLO port 1
  delay(60);
}



