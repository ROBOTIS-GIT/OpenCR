/* Serial2_Echo_interrupt
 
 Demonstrates sending data from the computer to the CM900, OpenCM9.04
 echoes it to the computer again.
 You can just type in terminal program, character you typed will be displayed
 
 You can connect the below products to J9 Connector in CM-900, OpenCM9.04
 [BT-110A] or [BT-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=875
 [ZIG-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=405
 [LN-101] download tool in CM-100
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=348
 
 You can also find all information about ROBOTIS products
 http://support.robotis.com/
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

void setup(){
  //Serial2 Serial initialize
  Serial2.begin(57600);
  //You can attach your serial interrupt
  //or, also detach the interrupt by detachInterrupt(void) method
  Serial2.attachInterrupt(serialInterrupt);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
}
void serialInterrupt(byte buffer){
  Serial2.print((char)buffer);
}
void loop(){
  toggleLED();
  delay(50);

}

