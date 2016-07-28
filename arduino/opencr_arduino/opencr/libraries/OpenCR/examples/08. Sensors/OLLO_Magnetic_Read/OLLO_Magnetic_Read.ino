/* OLLO Magnetic sensor Read example
 
 connect Magnetic Sensor Module to port 2.
 
  You can buy Magnetic Sensor DYNAMIXEL in ROBOTIS-SHOP
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
  myOLLO.begin(2, MAGNETIC_SENSOR);//OLLO Magnetic Module must be connected at port 2.
  
}
void loop(){
  SerialUSB.print("Magenetic Read = ");
  SerialUSB.println(myOLLO.read(2, MAGNETIC_SENSOR));
  delay(100); 
}
