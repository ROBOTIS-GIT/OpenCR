/*
 * Serial_HelloWorld
*/
volatile int nCount=0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  //print "Hello World!!" to PC though USB Virtual COM port
  Serial.println("Hello World!!");
  Serial.print("nCount : "); // display nCount variable and increase nCount.
  Serial.println(nCount++);
  
  delay(1000);
}


