/* Serial2_Echo
 
 Demonstrates sending data from the computer to the CM900, OpenCM9.04
 echoes it to the computer again.
 You can just type in terminal program, character you typed will be displayed
 
 [BT-110A] or [BT-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=875
 [ZIG-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=405
 [LN-101] download tool in CM-100
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=348
 
 You can also find all information about ROBOTIS products
 http://support.robotis.com/
 
                    Compatibility
 CM900                  X
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

/*
 Serial1 : Dynamixel_Poart
 Serial2 : Serial_Poart
 TxD(Cm9) <--(Connect)--> RxD(PC)
 RxD(Cm9) <--(Connect)--> TxD(PC)
*/


void setup(){
  //Serial2 Serial initialize
  Serial2.begin(57600);  
}
void loop(){
  // when you typed any character in terminal
  if(Serial2.available()){
    //print it out though USART2(RX2,TX2)
    Serial2.print((char)Serial2.read());
  }
}

