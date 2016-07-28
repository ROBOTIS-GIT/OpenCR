/* DigitalReadSerial

 Reads a digital input on pin 23, prints the result to the serial monitor 
 You can use BOARD_BUTTON_PIN instead of using pin 23 in OpenCM9.04
 
                Compatibility
CM900                  X
OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup(){
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
}
void loop(){
  int buttonState = digitalRead(BOARD_BUTTON_PIN);
  SerialUSB.print("buttonState = ");
  SerialUSB.println(buttonState);
  delay(100);
}


