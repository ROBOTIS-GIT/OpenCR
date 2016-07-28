/* OLLO Color sensor Read 

 When push OLLO Color sensor module, read digital I/O from the port 2
 
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#include <OLLO.h>
OLLO myOLLO;

void setup(){
  myOLLO.begin(2,COLOR_SENSOR);//OLLO Color Module must be connected at port 2.
  
}
void loop(){
  SerialUSB.print("COLOR Read = ");
  SerialUSB.println(myOLLO.read(2, COLOR_SENSOR));
  delay(100); 
}

/*  Result --->  myOLLO.read(2, COLOR_SENSOR) 
1 -> White
2 -> Black
3 -> Red
4 -> Green
5 -> Blue
6 -> Yellow
*/


