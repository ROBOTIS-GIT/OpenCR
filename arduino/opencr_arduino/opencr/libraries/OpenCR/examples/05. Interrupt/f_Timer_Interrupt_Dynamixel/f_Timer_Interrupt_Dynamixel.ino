/* Timer_interrupt_Dynamixel
  
  This example demonstrate how to use TIMER Interrupt with Dynamixel, 
  toggleLED.

                Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel Compatibility
               Ax    MX      Rx    XL-320    Pro
 CM900          O      O      O        O      O
 OpenCM9.04     O      O      O        O      O
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be OpenCM 485EXP board ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
*/
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM 1

/* Address_Number_Define */
#define Goal_Postion 30

#define LED_RATE 1000000    // in microseconds; should give 0.5Hz toggles
volatile unsigned int TimSec=0, DxlFlag=0, Pos=0;

Dynamixel Dxl(DXL_BUS_SERIAL1);
HardwareTimer Timer(1);// Instanciate HardwareTimer class on timer device 1

void setup() {
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3); 
  // Set up the LED to blink
  pinMode(BOARD_LED_PIN, OUTPUT);
  // Pause the timer while we're configuring it
  Timer.pause();

  // Set up period
  Timer.setPeriod(LED_RATE); // in microseconds

  // Set up an interrupt on channel 1
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, handler_led);

  // Refresh the timer's count, prescale, and overflow
  Timer.refresh();

  // Start the timer counting
  Timer.resume();
  
  Dxl.jointMode(ID_NUM);
}

void loop() {
  Dxl.writeWord(ID_NUM, Goal_Postion, Pos);
}

void handler_led(void) {
  toggleLED(); 
  Pos=Pos+100;
  if(Pos>1023)Pos=0;

}

