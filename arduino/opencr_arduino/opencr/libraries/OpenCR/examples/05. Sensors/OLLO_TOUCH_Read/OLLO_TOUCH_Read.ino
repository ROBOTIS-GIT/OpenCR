/* OLLO Touch sensor interrupt
 
 connect Touch Sensor Module to port 2.
 
  You can buy Touch Sensor DYNAMIXEL in ROBOTIS-SHOP
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
  myOLLO.begin(1,TOUCH_SENSOR);//OLLO Touch Module must be connected at port 2.
  
}
void loop(){
  Serial.print("Touch Read = ");
  Serial.println(myOLLO.read(1, TOUCH_SENSOR));
  delay(100); 
}




