/*SerialUSB_Serial2_Converter
 
 This example is convert from serial2 to USB.
 CM-900, OpenCM9.04 has a port(J9) connected directly to Serial2.
 If some data is coming from Serial2, it is sent to serialUSB.
 On the contrary, all data coming from serialUSB is sent to Serial2.
 
 
 You can connect the below products to J9 Connector in CM-900, OpenCM9.04
 [BT-110A] or [BT-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=875
 [ZIG-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=405
 [LN-101] USART communication and download tool in CM-100
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
  Serial2.begin(57600);  
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop(){
  if(SerialUSB.available()){
    Serial2.print((char)SerialUSB.read());//send data coming from USB to Serial2
  }

  if(Serial2.available()){
    toggleLED();
    SerialUSB.print((char)Serial2.read()); //send data coming from Serial2 to USB(PC)
  }
}

