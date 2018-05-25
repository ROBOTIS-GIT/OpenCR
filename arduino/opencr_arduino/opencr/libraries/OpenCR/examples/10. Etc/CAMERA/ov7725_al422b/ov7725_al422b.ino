#include "ov7725_al422b.h"
#include "settings.h"

HardwareTimer Timer(TIMER_CH1);
uint8_t fps = 0;
uint8_t frameCount = 0;
uint8_t size = QVGA;

uint32_t panGoalPosition = 2048;
uint32_t tiltGoalPosition = 2048;
uint32_t panPresentPosition = 0;
uint32_t tiltPresentPosition = 0;
uint8_t  dxlError = 0;
int      dxlCommResult = COMM_TX_FAIL;

void setup() {
  uint8_t ov7725Vsync = 0;
  
  Serial.begin(115200);
  
  dynamixel::PortHandler *hPort = dynamixel::PortHandler::getPortHandler(DXL_PORT);
  dynamixel::PacketHandler *hPacket = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  initDynamixel(hPort, hPacket);

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
      findCenterCell();
      displayInfo(fps);
      drawScreen();
      trackObject(hPort, hPacket);
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