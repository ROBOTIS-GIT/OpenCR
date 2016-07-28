/* Scratch_Sensor_Board_Emulator
 
 A program that simulates a Scratch board using an Arduino. This version reads the button and  inputs A-D.
 Version 0.6
 
                 Compatibility
 CM900                  X
 OpenCM9.04             O
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

#include <OLLO.h>
OLLO myOLLO;

void ScratchBoardSensorReport(int sensor, int value)
{
  SerialUSB.print( B10000000
    | ((sensor & B1111)<<3)
    | ((value>>7) & B111),BYTE);
  SerialUSB.print( value & B1111111, BYTE);
}

void setup()
{

  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);
  myOLLO.begin(1,TOUCH_SENSOR);
  myOLLO.begin(2, IR_SENSOR);
  myOLLO.begin(3);
  myOLLO.begin(4);

}
int irSensor = 0;
int gyroX=0;
int gyroY=0;
void loop() 
{ 

  if(SerialUSB.available()){
    gyroX = myOLLO.read(3);
    if(gyroX > 1000 && gyroX < 2000){
      gyroX = 1530;
    }
    gyroY = myOLLO.read(4);
    if(gyroY > 1000 && gyroY < 2000){
      gyroY = 1530;
    }
    ScratchBoardSensorReport(0, map(gyroX, 0, 3000, 0, 1023) );
    ScratchBoardSensorReport(1, map(gyroY, 0, 3000, 0, 1023) );
    ScratchBoardSensorReport(2, 0/*analogRead(3)*/);
    ScratchBoardSensorReport(3, 0/*analogRead(4)*/);
    irSensor = myOLLO.read(2, IR_SENSOR);
    if(irSensor > 250)
      irSensor = 300;
    ScratchBoardSensorReport(4, map(irSensor, 0, 300, 0, 1023));
    ScratchBoardSensorReport(5, 0);
    ScratchBoardSensorReport(6, 0);
    ScratchBoardSensorReport(7, (myOLLO.read(1, TOUCH_SENSOR) ? 1023 : 0));
    // Let Scratch catch up with us
    delay(30);
    toggleLED();
  }
}










