/* Dynamixel Bus Scanner 
 
  Searches through all valid IDs to find all dynamixel devices currently on
  the bus, and uses the Model Number to identify the device by name
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
  **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
 
// My gigantic dynamixel header file
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);

  pinMode(BOARD_LED_PIN, OUTPUT);

  // Waits 5 seconds for you to open the console (open too quickly after
  //   downloading new code, and you will get errors
  delay(5000);
  SerialUSB.print("Send any value to continue...\n");
  while(!SerialUSB.available())
  {
    delay(1000);
    digitalWrite(BOARD_LED_PIN, LOW);
    SerialUSB.print("Send any value to continue...\n");
    delay(1000);
    digitalWrite(BOARD_LED_PIN, HIGH);
  }
}
int model;
void loop() {
  // put your main code here, to run repeatedly: 
  for (int i=1; i<50; i++){
    SerialUSB.print(i);
    delay(10);

    model = Dxl.readWord(i, 0);

    if(model == 12)
      SerialUSB.println(": AX-12A");

    else if(model == 300)
      SerialUSB.println(": AX-12W");

    else if(model == 18)
      SerialUSB.println(": AX-18A");

    else if(model == 29)
      SerialUSB.println(": MX-28");     

    else if(model == 54)
      SerialUSB.println(": MX-64");

    else if(model == 64)
      SerialUSB.println(": MX-106");

    else if(model == 350)
      SerialUSB.println(": XL-320");   

    else{
      if(model == 65535) model = 0;
      SerialUSB.print(": Unknown : "); 
      SerialUSB.println(model);
    }
  }
  
  while(1){
    digitalWrite(BOARD_LED_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_LED_PIN, HIGH);
    delay(100);
  }
}

