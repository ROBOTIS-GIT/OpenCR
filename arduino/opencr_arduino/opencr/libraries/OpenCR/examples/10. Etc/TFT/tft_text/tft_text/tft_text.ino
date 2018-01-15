#include <LcdTouchPanel.h>

void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color){
  Tft.lcd_display_string(x_pos, y_pos, string, ch_size, color) ;
}
void drawFrame(){
  Tft.drawFrame();
}

////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tftInit();
}

void loop() {
  static uint32_t pre_time;

  if(millis() - pre_time >= 50)
  {
    drawText(40,40, (const uint8_t *)"Hello, World!!!",16, RED);
    drawFrame();
  }
}
