
#include "ov7725_al422b.h"

#define BOARD_LED_PIN           BDPIN_LED_STATUS    //Status LED
#define LED_RATE                500000              // in microseconds; should toggle every 0.5sec
#define OPENCR_OV7725   //Select I2C2 on GPIO pins

HardwareTimer Timer(TIMER_CH1);
uint8_t fps = 0;
char fpsString[2];


////////////////////////////////////
void setup() {
  uint8_t ov7725Vsync = 0;

  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BDPIN_LED_USER_1, OUTPUT);
  pinMode(BDPIN_LED_USER_2, OUTPUT);
  pinMode(BDPIN_LED_USER_3, OUTPUT);
  pinMode(BDPIN_LED_USER_4, OUTPUT);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  // memcpy(image_buf, BLACK, 160*120);
    
  Timer.stop();
  Timer.setPeriod(LED_RATE);
  Timer.attachInterrupt(status_led);
  Timer.start();

  // put your setup code here, to run once:
  Serial.begin(115200);
  
  lcdInit();
  
  swapXY();
  setGRamArea(QVGA, true);
  setRawImgSize(QVGA);

  OV7725_Init();

  while(1)
  {
    ov7725Vsync = getVsync();
    if(ov7725Vsync == 2)
    {
      readIMG();
      drawText(10, 20, (const uint8_t *)fpsString, 16, RED);
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