/*Serial3_Echo
 
 Demonstrates sending data from the computer to the CM900, OpenCM9.04
 echoes it to the computer again.
 You can just type in terminal program, character you typed will be displayed
 
 You can connect the below products to J9 Connector in CM900, OpenCM9.04
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

/*
Serial1 : Dynamixel_Poart
 Serial2 : Serial_Poart(4pin_Molex)
 Serial3 : Serial_Poart(pin26:Tx3, pin27:Rx3)
 
 TxD3(Cm9_Pin26) <--(Connect)--> RxD(PC)
 RxD3(Cm9_Pin27) <--(Connect)--> TxD(PC)
 */


void setup(){
  //Serial3 Serial initialize
  Serial3.begin(57600);  
}
void loop(){
  // when you typed any character in terminal
  if(Serial3.available()){
    //print it out though USART2(RX2,TX2)
    Serial3.print((char)Serial3.read());
  }
}

