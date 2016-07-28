/*SerialUSB_Echo_Interrupt
 
 Demonstrates sending data from the computer to the CM900,OpenCM9.04
 echoes it to the computer again.
 You can just type in terminal program, character you typed will be displayed

                 Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup(){
  //You can attach your serialUSB interrupt
  //or, also detach the interrupt by detachInterrupt(void) method
  SerialUSB.attachInterrupt(usbInterrupt);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
}

void usbInterrupt(byte* buffer, byte nCount){
  SerialUSB.print("nCount =");
  SerialUSB.println(nCount);
  for(unsigned int i=0; i < nCount;i++)  //printf_SerialUSB_Buffer[N]_receive_Data
    SerialUSB.print((char)buffer[i]);
  SerialUSB.println("");
}

void loop(){
  toggleLED();
  delay(100);

}

