/*
 *******************************************************************************
 *  IR Sensor Array Example (CM-900 version)
 *******************************************************************************
 *  The program reads the IR sensor pin states and uses those to control the
 *    tones played by the IR Sensor Array and a single AX-12W speed.
 *      IR4 stops rotation.
 *      IR1==4, IR2==2, IR3==1 => Summed and saved as variable 'left'
 *      IR7==4, IR6==2, IR5==1 => Summed and saved as variable 'right'
 *    if ((IR4 == 1) || (left==right)), then speed = 0
 *    if (left > right), then speed = (left-right)*146 in CCW direction
 *    if (right > left), then speed = (right-left)*146 in CW direction
 *******************************************************************************
 *  2014-05-22 modified by ROBOTIS CO,.LTD.
 */
// My gigantic dynamixel header file
#include <HaViMo2.h>
  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

#define DXL_WHEEL_ID	2
#define DXL_IRSA_ID	100

void setup()
{
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  SerialUSB.begin();

  // Waits 10 seconds for you to open the console (open too quickly after
  //   downloading new code, and you will get errors
  delay(10000);
  SerialUSB.print("Send any value to continue...\n");
  while(!SerialUSB.available())
  {
    delay(1000);
    digitalWrite(BOARD_LED_PIN, LOW);
    SerialUSB.print("Send any value to continue...\n");
    delay(1000);
    digitalWrite(BOARD_LED_PIN, HIGH);
  }
  SerialUSB.print("Now starting program\n");

  Dxl.writeWord(DXL_WHEEL_ID, AXM_CW_ANGLE_LIMIT_L, 0);
  Dxl.writeWord(DXL_WHEEL_ID, AXM_CCW_ANGLE_LIMIT_L, 0);
  Dxl.writeByte(DXL_WHEEL_ID, AXM_TORQUE_ENABLE, 1);
  Dxl.writeWord(DXL_WHEEL_ID, AXM_MOVING_SPEED_L, 0);

  delay(2000);
}
void loop()
{
  int ir=0;
  while(1) 

    ir = Dxl.readByte(DXL_IRSA_ID, IRSA_OBS_DET);

    // Print out in order as labeled on top of IRSA
    SerialUSB.print("    1234567\n    ");

    int i;
    int count=0;

    for (i=0; i<7; i++)
    {
      if (ir&(1<<i))
        count++;
    }

    for (i=0; i<7; i++)
    {
      if (ir&(1<<i))
      {
        // Print 1 if high/black detected
        SerialUSB.print("1");
        // Play note for 0.1 second
        Dxl.writeByte(DXL_IRSA_ID, IRSA_BUZZ_TIME, (1));
        // Play note based on IR sensor detecting line
        Dxl.writeByte(DXL_IRSA_ID, IRSA_BUZZ_NOTE, (7+(1*i)));
        // Delay while note plays
        delay(100);
      }
      else
      {
        SerialUSB.print("0");
      }

    }
    SerialUSB.print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    int speed = 0;
    // IR 1 2 3		extreme/large to center/small
    int left = ((ir<<2)&0x04) | ((ir)&0x02) | ((ir>>2)&0x01);
    // IR 7 6 5		extreme/large to center/small
    int right = (ir>>4)&0x07;
    if ((ir&0x08))
    {
      speed = 0;
    }
    else if (left > right)
    {
      speed = ((left-right) * 146);
    }
    else if (right > left)
    {
      // OR by 0x0400 to change direction to CW
      speed = ((right-left) * 146) | (0x0400);
    }
    else
    {
      speed = 0;
    }
    Dxl.writeWord(DXL_WHEEL_ID, AXM_MOVING_SPEED_L, speed);
}

