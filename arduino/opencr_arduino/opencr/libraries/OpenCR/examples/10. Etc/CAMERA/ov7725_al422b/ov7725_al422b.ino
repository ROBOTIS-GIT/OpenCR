
#include "ov7725_al422b.h"

#define BOARD_LED_PIN           BDPIN_LED_STATUS    //Status LED
#define LED_RATE                500000              // in microseconds; should toggle every 0.5sec
#define _USE_FULLSCREEN         false


HardwareTimer Timer(TIMER_CH1);
uint8_t fps = 0;
uint8_t size = QVGA;
char fpsString[3];

////////////////////////////////////
void setup() {
  uint8_t ov7725Vsync = 0;
  if(_USE_FULLSCREEN == true)
  {
    size = QVGA;
  }
  else
  {
    size = QUARTERVIEW;
  }

  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BDPIN_LED_USER_1, OUTPUT);
  pinMode(BDPIN_LED_USER_4, OUTPUT);
  pinMode(BDPIN_LED_USER_2, OUTPUT);
  pinMode(BDPIN_LED_USER_3, OUTPUT);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  // memcpy(image_buf, BLACK, 160*120);
    
  Timer.stop();
  Timer.setPeriod(LED_RATE);
  Timer.attachInterrupt(status_led);
  Timer.start();

  // put your setup code here, to run once:
  Serial.begin(115200);
  
  lcdInit();
  
  lcdRotation(3);
  setGRamArea(size);

  OV7725_Init();

  while(1)
  {
    ov7725Vsync = getVsync();
    if(ov7725Vsync == 2)
    {
      readIMG(size);
      // colorFilter(image_buf, size);
      // objectFinder(color_filter);
      // cellWeight();
      //display text information
      drawText(20, 5, (const uint8_t *)"fps", 12, RED);
      drawText(5, 5, (const uint8_t *)fpsString, 12, RED);
      TFTLCD.drawFrame();
      setVsync(0);
      fps++;
    }
  }
}


void status_led(void) {
  static uint8_t flag = 0;
  digitalWrite(BOARD_LED_PIN, flag);
  flag ^= 1;
  if(flag == 0)
  {
    itoa(fps, fpsString, 10);
    fps = 0;
  }
}

void loop() {
  
}