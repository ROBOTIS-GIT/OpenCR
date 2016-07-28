/*
*   Whatever it receives from the Serial USB (usually commands and queries) 
 *  is sent to the Dynamixel bus, and what it receives from the Dynamixel bus
 *  is sent to the SerialUSB (usually answers):
 *  Created by aprendiendo
 *  http://softwaresouls.com/softwaresouls/2013/06/12/robotis-cm-900-as-a-tosser-for-dynamixel-commands/
	1. Connect XL-320 to OpenCM9.04 or CM-900
	2. Go to menu File > Example > DYNAMIXEL > Tosser 
	3. Now your OpenCM 9.04 is ready to works just like USB2DYNAMIXEL
	4. Run RoboPlus > DYNAMIXEL Wizard
	5. Connect to OpenCM USB COM port and then you can change control table of dynamixel.
 *  2014-05-22 modified by ROBOTIS CO,,LTD.
 */

  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

int counter;
bool onlyOnceHappened;

void blinkOnce()
{
  digitalWrite(BOARD_LED_PIN, LOW);
  delay_us(100);
  digitalWrite(BOARD_LED_PIN, HIGH);
}

void setup()
{  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  onlyOnceHappened=false;
  counter=0;
}

byte aByte=0;
uint8 aUint8;

void loop() 
{    
  if (onlyOnceHappened==false)
  {    
    blinkOnce();
    onlyOnceHappened=true;
    delay (3000); //Some time to the user to activate the monitor/console
    SerialUSB.println ("v1.1.1 Orders receiver started");  
  }    

  if (SerialUSB.available())
  {
    aUint8=SerialUSB.read();
    blinkOnce();
    Dxl.writeRaw(aUint8);
    //    delay(20);
  }   

  if (Dxl.available())
  {
    aByte=Dxl.readRaw();
    blinkOnce();
    SerialUSB.write(aByte);
  } 
}



