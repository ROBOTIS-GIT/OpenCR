#include <LcdTouchPanel.h>


void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color){
  switch(shape)
  {
    case CIRCLE:
     Tft.lcd_draw_circle(x_pos, y_pos, radius, color);
     break;
     
    case SQUARE:
     Tft.lcd_fill_rect(x_pos, y_pos, radius, radius, color);
     break;
  }
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
    drawShape(CIRCLE, 120, 80, 50, RED);
    drawShape(SQUARE, 120, 160, 50, RED);
    drawFrame();
  }
}
