
#include "ov7725_al422b.h"
#include "settings.h"

HardwareTimer Timer(TIMER_CH1);
uint8_t fps = 0;
uint8_t frameCount = 0;
uint8_t size = QVGA;

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
      colorFilter(image_buf, size);
      objectFinder(masked_image_buf);
      cellWeight();
      displayCell();
      //display text information
      displayInfo(fps);
      drawScreen();
      setVsync(0);
      frameCount++;
    }
  }
}


void status_led(void) {
  static uint8_t flag = 0;
  digitalWrite(BOARD_LED_PIN, flag);
  flag ^= 1;
  if(flag == 0)
  {
    fps = frameCount;
    frameCount = 0;
  }
}

void loop() {
  
}