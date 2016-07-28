/* SerialUSB_HelloWorld

 USB Serial print "Hello World!!" to PC 
 You can see it any terminal program, putty, TeraTerm, Hyper Terminal, etc...
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O

 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
*/
volatile int nCount=0;

void setup() {
}

void loop() {
  //print "Hello World!!" to PC though USB Virtual COM port
  SerialUSB.println("Hello World!!");
  SerialUSB.print("nCount : "); // display nCount variable and increase nCount.
  SerialUSB.println(nCount++);//SerialUSB.print("\r\n");  
  
  delay(1000);
}


