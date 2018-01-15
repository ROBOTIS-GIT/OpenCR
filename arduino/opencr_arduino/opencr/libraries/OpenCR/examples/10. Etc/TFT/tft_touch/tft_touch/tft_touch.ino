//Before, running this project. you have to know that the coordinate of LCD and Touch is different. 
//The relationship between LCD and touch is,
//LCD   x : 0 ~ 240,    y : 0 ~ 320   (origin : when you set the OpenCR direction with the usb port facing downside, left-upper point is (0,0))
//Touch x : 1950 ~ 127, y : 1950 ~ 127
//So, I'am gonna support you by using convCoordinateTouch2LCD function

#include <LcdTouchPanel.h>

uint16_t lcd_list[2];
uint16_t* convCoordinateTouch2LCD(uint16_t* touch_list)
{
   lcd_list[XPOS] = LCD_MAX_X*(TOUCH_MAX-touch_list[XPOS])/TOUCH_RANGE;
   lcd_list[YPOS] = LCD_MAX_Y*(TOUCH_MAX-touch_list[YPOS])/TOUCH_RANGE;
   return lcd_list;
}


void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color)
{
  Tft.lcd_display_string(x_pos, y_pos, string, ch_size, color) ;
}

void drawTouch(uint16_t x_pos, uint16_t y_pos, uint16_t color)
{
  Tft.lcd_draw_circle(x_pos, y_pos, 5, color);
}

void drawFrame()
{
  Tft.drawFrame();
}

////////////////////////////////////
uint16_t touch_pos[2];


void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  tftInit();    
}

void loop()
{
  // put your main code here, to run repeatedly:
  static uint32_t pre_time;
  uint16_t *lcd_pos;
 
  getSingleTouchPoint(touch_pos);
  lcd_pos = convCoordinateTouch2LCD(touch_pos);

  if(millis() - pre_time >= 50)
  {
    pre_time = millis();
    Serial.print("touch : ");
    Serial.print(touch_pos[XPOS]);
    Serial.print(" , ");
    Serial.println(touch_pos[YPOS]);
    Serial.print("lcd : ");
    Serial.print(lcd_pos[XPOS]);
    Serial.print(" , ");
    Serial.println(lcd_pos[YPOS]);
    Serial.println();

    drawText(40,40, (const uint8_t *)"Touch The Screen !!",16, RED);
    drawTouch(lcd_pos[XPOS], lcd_pos[YPOS], RED);
    drawFrame();
  }
}
