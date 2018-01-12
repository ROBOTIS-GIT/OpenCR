#include "logo.h"
#include <RTOS.h>

void setup(void) 
{
  Serial.begin(115200);
  while(!Serial);  
  logo_drawLogo();  


  pinMode(13, OUTPUT);
}

void loop() 
{
  digitalWrite(13, !digitalRead(13));
  Serial.println("test");
  delay(100);
}


