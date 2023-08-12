// OLLO Color Sensor(CS-10) example
// 
// Connect Color Sensor Module(CS-10) to OLLO port 1 in the OpenCR.
// 
// You can buy Color Sensor in ROBOTIS-SHOP
// http://www.robotis-shop-en.com/shop/step1.php?number=750&b_code=B20070914051413&c_code=C20100528062452
// You can also find all information 
// http://support.robotis.com/
//                
// This example is tested with the following device
// Controller : OpenCR
// Sensor : CS-10
// 
// Created : 13 July 2023
// by YKW. ROBOTIS Co., LTD.

#include <OLLO.h>
OLLO myOLLO;

void setup() {
  myOLLO.begin(1, COLOR_SENSOR); // OLLO Color Module must be connected at port 1.
  Serial.begin(115200);
}

void loop() {
  int colorValue = myOLLO.read(1, COLOR_SENSOR);

  Serial.print("COLOR Sensor Read = ");
  switch (colorValue) {
    case 0:
      Serial.println("Unknown color");
      break;
    case 1:
      Serial.println("White");
      break;
    case 2:
      Serial.println("Black");
      break;
    case 3:
      Serial.println("Red");
      break;
    case 4:
      Serial.println("Green");
      break;
    case 5:
      Serial.println("Blue");
      break;
    case 6:
      Serial.println("Yellow");
      break;
    default:
      Serial.println("Unknown color");
      break;
  }

  delay(100);
}